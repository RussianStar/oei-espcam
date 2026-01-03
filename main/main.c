#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

// ----- XIAO ESP32S3 Sense pin map (Seeed table) -----
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39

#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15

#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

static const char *TAG = "snap_usb";

static const uint8_t MAGIC[4] = {'J', 'P', 'G', '0'};
static const uint8_t END[4] = {'E', 'N', 'D', '0'};

// OV2640 cannot do true 1920x1080; use UXGA (1600x1200) max.
// If you install OV5640/OV3660, set FRAMESIZE_FHD.
#ifndef SNAP_FRAME_SIZE
#define SNAP_FRAME_SIZE FRAMESIZE_UXGA
#endif

static bool cam_ok = false;

static esp_err_t camera_init_once(void) {
    if (cam_ok) {
        return ESP_OK;
    }

    camera_config_t c = {0};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer = LEDC_TIMER_0;

    c.pin_d0 = Y2_GPIO_NUM;
    c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;
    c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;
    c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;
    c.pin_d7 = Y9_GPIO_NUM;

    c.pin_xclk = XCLK_GPIO_NUM;
    c.pin_pclk = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href = HREF_GPIO_NUM;
    c.pin_sscb_sda = SIOD_GPIO_NUM;
    c.pin_sscb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;

    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size = SNAP_FRAME_SIZE;

    c.jpeg_quality = 12;
    c.fb_count = 1;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }
    cam_ok = true;
    return ESP_OK;
}

static void camera_deinit(void) {
    if (cam_ok) {
        esp_camera_deinit();
        cam_ok = false;
    }
}

static void write_all_usb(const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        int w = usb_serial_jtag_write_bytes(p, len, pdMS_TO_TICKS(1000));
        if (w > 0) {
            p += (size_t)w;
            len -= (size_t)w;
            continue;
        }
        if (w == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        ESP_LOGW(TAG, "usb write error: %d", w);
        return;
    }
}

static void write_u32_le(uint32_t v) {
    uint8_t b[4] = {
        (uint8_t)v,
        (uint8_t)(v >> 8),
        (uint8_t)(v >> 16),
        (uint8_t)(v >> 24),
    };
    write_all_usb(b, sizeof(b));
}

static void do_snap(void) {
    if (camera_init_once() != ESP_OK) {
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "esp_camera_fb_get failed");
        return;
    }

    // Frame: MAGIC + LEN + JPEG + END
    write_all_usb(MAGIC, sizeof(MAGIC));
    write_u32_le((uint32_t)fb->len);
    write_all_usb(fb->buf, fb->len);
    write_all_usb(END, sizeof(END));

    esp_camera_fb_return(fb);

    // Deinit to reduce idle load/heat between shots
    camera_deinit();
}

static void camera_self_test(void) {
    ESP_LOGI(TAG, "camera self-test: init");
    if (camera_init_once() != ESP_OK) {
        ESP_LOGE(TAG, "camera self-test: init failed");
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "camera self-test: fb_get failed");
        camera_deinit();
        return;
    }

    ESP_LOGI(TAG, "camera self-test: ok, len=%u format=%d size=%d",
             (unsigned)fb->len, fb->format, fb->width * fb->height);
    esp_camera_fb_return(fb);
    camera_deinit();
    ESP_LOGI(TAG, "camera self-test: done");
}

static void cmd_task(void *arg) {
    (void)arg;
    char line[32];
    size_t n = 0;

    // stdin over USB Serial/JTAG can occasionally return 0 bytes; ignore and retry.
    while (1) {
        uint8_t ch;
        int r = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(20));

        if (r > 0) {
            if (ch == '\n' || ch == '\r') {
                line[n] = 0;
                if (strcmp(line, "SNAP") == 0) {
                    do_snap();
                }
                n = 0;
            } else if (n + 1 < sizeof(line)) {
                line[n++] = (char)ch;
            }
            continue;
        }

        if (r == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ESP_LOGW(TAG, "usb read error: %d", r);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {
    // Install USB Serial/JTAG driver + route stdio via VFS.
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 4096,
        .rx_buffer_size = 4096,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    usb_serial_jtag_vfs_use_driver();

    setvbuf(stdout, NULL, _IONBF, 0); // unbuffered stdout for binary framing

    printf("READY snap_usb v1\n");
    printf("Send: SNAP\\n\n");

    camera_self_test();

    xTaskCreate(cmd_task, "cmd_task", 4096, NULL, 10, NULL);
}
