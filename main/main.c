// main.c — XIAO ESP32S3 Sense (esp-idf) SNAP over USB Serial/JTAG
// Adds "log everything to USB" while keeping JPG0/LEN/JPEG/END0 frames uncorrupted
// by buffering log output during frame transmission and flushing after END0.

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "esp_camera.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#include "driver/i2c_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_dev.h"

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
static const uint8_t END[4]   = {'E', 'N', 'D', '0'};

#ifndef CAM_I2C_PORT
#define CAM_I2C_PORT I2C_NUM_1
#endif

#ifndef SNAP_I2C_PROBE
#define SNAP_I2C_PROBE 1
#endif

#ifndef SNAP_AUTOSNAP
#define SNAP_AUTOSNAP 0
#endif

#ifndef SNAP_LOGS_ONLY
#define SNAP_LOGS_ONLY 0
#endif

// OV3660 max QXGA, OV5640 can go higher.
// For OV2640, set FRAMESIZE_UXGA.
#ifndef SNAP_FRAME_SIZE
#define SNAP_FRAME_SIZE FRAMESIZE_QXGA
#endif

#ifndef SNAP_JPEG_QUALITY
#define SNAP_JPEG_QUALITY 12
#endif

#ifndef SNAP_DEINIT
#define SNAP_DEINIT 0
#endif

#define USB_WRITE_CHUNK 1024

// -------------------- USB raw writer (never logs) --------------------
static void usb_write_raw(const void *buf, size_t len) {
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        size_t to_write = (len > USB_WRITE_CHUNK) ? USB_WRITE_CHUNK : len;
        int w = usb_serial_jtag_write_bytes(p, to_write, pdMS_TO_TICKS(2000));
        if (w > 0) {
            p += (size_t)w;
            len -= (size_t)w;
            continue;
        }
        if (w == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        // error: drop silently (avoid recursion via ESP_LOG)
        return;
    }
}

static void write_u32_le(uint32_t v) {
    uint8_t b[4] = {
        (uint8_t)(v),
        (uint8_t)(v >> 8),
        (uint8_t)(v >> 16),
        (uint8_t)(v >> 24),
    };
    usb_write_raw(b, sizeof(b));
}

// -------------------- Log-to-USB with frame-safe buffering --------------------
static volatile bool g_in_frame = false;

static char g_log_buf[8192];
static size_t g_log_len = 0;
static portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static int usb_log_vprintf(const char *fmt, va_list ap) {
    char tmp[256];
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    if (n <= 0) return n;

    size_t len = (size_t)n;
    if (len >= sizeof(tmp)) len = sizeof(tmp) - 1;

    if (g_in_frame) {
        portENTER_CRITICAL(&g_log_mux);
        size_t room = sizeof(g_log_buf) - g_log_len;
        size_t copy = (len < room) ? len : room;
        if (copy) {
            memcpy(g_log_buf + g_log_len, tmp, copy);
            g_log_len += copy;
        }
        portEXIT_CRITICAL(&g_log_mux);
    } else {
        usb_write_raw(tmp, len);
    }
    return n;
}

static void flush_log_buf(void) {
    char local[sizeof(g_log_buf)];
    size_t len = 0;

    portENTER_CRITICAL(&g_log_mux);
    len = g_log_len;
    if (len) {
        memcpy(local, g_log_buf, len);
        g_log_len = 0;
    }
    portEXIT_CRITICAL(&g_log_mux);

    if (len) {
        usb_write_raw(local, len);
    }
}

static void soft_reset_now(void) {
    g_in_frame = false;
    flush_log_buf();
    static const char msg[] = "RESETTING\n";
    usb_write_raw(msg, sizeof(msg) - 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_restart();
}

// -------------------- Camera init/deinit --------------------
static bool cam_ok = false;

static bool psram_ready(void) {
#if CONFIG_SPIRAM
    return esp_psram_is_initialized();
#else
    return false;
#endif
}

static bool i2c_probe_camera_bus(void) {
#if !SNAP_I2C_PROBE
    return true;
#else
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = CAM_I2C_PORT,
        .sda_io_num = SIOD_GPIO_NUM,
        .scl_io_num = SIOC_GPIO_NUM,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "i2c: init failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t found = 0;
    const uint8_t addrs[] = {0x30, 0x3c}; // OV2640 / OV5640 typical
    for (size_t i = 0; i < sizeof(addrs); ++i) {
        uint8_t addr = addrs[i];
        if (i2c_master_probe(bus, addr, pdMS_TO_TICKS(100)) == ESP_OK) {
            ESP_LOGI(TAG, "i2c: device found at 0x%02x", addr);
            found = 1;
        }
    }

    i2c_del_master_bus(bus);

    if (!found) {
        ESP_LOGW(TAG, "i2c: no devices found on camera bus");
    }
    return found != 0;
#endif
}

static esp_err_t camera_init_once(void) {
    if (cam_ok) return ESP_OK;

    ESP_LOGI(TAG, "camera init: start");

    if (!i2c_probe_camera_bus()) {
        ESP_LOGW(TAG, "camera init: i2c probe failed / no sensor");
        // still attempt esp_camera_init; some sensors may not ACK until powered/clocked
    }

    camera_config_t c = {0};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;

    c.pin_d0 = Y2_GPIO_NUM;
    c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;
    c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;
    c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;
    c.pin_d7 = Y9_GPIO_NUM;

    c.pin_xclk  = XCLK_GPIO_NUM;
    c.pin_pclk  = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM;
    c.pin_href  = HREF_GPIO_NUM;

    c.pin_sccb_sda = SIOD_GPIO_NUM;
    c.pin_sccb_scl = SIOC_GPIO_NUM;
    c.sccb_i2c_port = CAM_I2C_PORT;

    c.pin_pwdn  = PWDN_GPIO_NUM;
    c.pin_reset = RESET_GPIO_NUM;

    c.xclk_freq_hz = 20000000;
    c.pixel_format = PIXFORMAT_JPEG;
    c.frame_size   = SNAP_FRAME_SIZE;

    c.jpeg_quality = SNAP_JPEG_QUALITY;
    c.fb_count     = 1;
    c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    if (psram_ready()) {
        c.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        ESP_LOGW(TAG, "psram not detected; using DRAM + FRAMESIZE_VGA");
        c.fb_location = CAMERA_FB_IN_DRAM;
        c.frame_size  = FRAMESIZE_VGA;
    }

    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    cam_ok = true;
    ESP_LOGI(TAG, "camera init: ok");
    return ESP_OK;
}

static void camera_deinit(void) {
    if (cam_ok) {
        ESP_LOGI(TAG, "camera deinit");
        esp_camera_deinit();
        cam_ok = false;
    }
}

// -------------------- Framed output helpers --------------------
static void send_empty_frame(void) {
    g_in_frame = true;
    usb_write_raw(MAGIC, sizeof(MAGIC));
    write_u32_le(0);
    usb_write_raw(END, sizeof(END));
    g_in_frame = false;
    flush_log_buf();
}

// -------------------- SNAP operation --------------------
static void do_snap(void) {
    ESP_LOGI(TAG, "snap: begin t_us=%" PRIu64, (uint64_t)esp_timer_get_time());

    esp_err_t init_err = camera_init_once();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "snap: camera init failed: %s", esp_err_to_name(init_err));
        send_empty_frame();
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "snap: esp_camera_fb_get failed");
        send_empty_frame();
        return;
    }

    ESP_LOGI(TAG, "snap: fb len=%u fmt=%d %ux%u",
             (unsigned)fb->len, fb->format, fb->width, fb->height);

    // Binary frame: JPG0 + LEN + JPEG + END0
    g_in_frame = true;
    usb_write_raw(MAGIC, sizeof(MAGIC));
    write_u32_le((uint32_t)fb->len);
    usb_write_raw(fb->buf, fb->len);
    usb_write_raw(END, sizeof(END));
    g_in_frame = false;

    esp_camera_fb_return(fb);

    flush_log_buf();

    if (SNAP_DEINIT) {
        camera_deinit();
    }

    ESP_LOGI(TAG, "snap: done");
}

// -------------------- Self-test --------------------
static void camera_self_test(void) {
    bool psram_ok = psram_ready();
#if CONFIG_SPIRAM
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
#else
    size_t psram_free = 0;
#endif
    ESP_LOGI(TAG, "psram: %s, free=%u bytes",
             psram_ok ? "ok" : "not detected", (unsigned)psram_free);

    ESP_LOGI(TAG, "self-test: init");
    if (camera_init_once() != ESP_OK) {
        ESP_LOGE(TAG, "self-test: init failed");
        return;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "self-test: fb_get failed");
        camera_deinit();
        return;
    }

    ESP_LOGI(TAG, "self-test: ok len=%u fmt=%d %ux%u",
             (unsigned)fb->len, fb->format, fb->width, fb->height);

    esp_camera_fb_return(fb);

    if (SNAP_DEINIT) camera_deinit();
    ESP_LOGI(TAG, "self-test: done");
}

// -------------------- Command task --------------------
// Commands: SNAP, PING, SLEEP
static void cmd_task(void *arg) {
    (void)arg;
    char line[32];
    size_t n = 0;

    ESP_LOGI(TAG, "cmd_task: start");

    while (1) {
        uint8_t ch;
        int r = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(20));
        if (r > 0) {
            if (ch == '\n' || ch == '\r') {
                line[n] = 0;

                if (n == 0) {
                    // ignore empty lines
                } else if (strcmp(line, "SNAP") == 0) {
                    static const char ack[] = "ACK\n";
                    usb_write_raw(ack, sizeof(ack) - 1);
                    do_snap();
                } else if (strcmp(line, "PING") == 0) {
                    static const char ack[] = "ACK\n";
                    usb_write_raw(ack, sizeof(ack) - 1);
                    ESP_LOGI(TAG, "PING");
                } else if (strcmp(line, "RESET") == 0 || strcmp(line, "REBOOT") == 0) {
                    soft_reset_now();
                } else if (strcmp(line, "SLEEP") == 0) {
                    static const char ack[] = "ACK\n";
                    usb_write_raw(ack, sizeof(ack) - 1);
                    camera_deinit();
                    ESP_LOGI(TAG, "SLEEP (camera deinit only)");
                } else {
                    ESP_LOGW(TAG, "unknown cmd: %s", line);
                }

                n = 0;
            } else if (n + 1 < sizeof(line)) {
                line[n++] = (char)ch;
            } else {
                // overflow: reset line
                n = 0;
            }
            continue;
        }

        if (r == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        ESP_LOGW(TAG, "usb read error: %d", r);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -------------------- app_main --------------------
void app_main(void) {
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 8192,
        .rx_buffer_size = 8192,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    esp_vfs_usb_serial_jtag_use_driver();

    // Route ESP_LOGx to our USB writer (frame-safe buffering).
    esp_log_set_vprintf(usb_log_vprintf);

    // Unbuffer stdio.
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    // Enable maximum logging.
    esp_log_level_set("*", ESP_LOG_VERBOSE);

    static const char boot_msg[] = "BOOT\n";
    usb_write_raw(boot_msg, sizeof(boot_msg) - 1);
    ESP_LOGI(TAG, "reset reason: %d", esp_reset_reason());

    printf("READY snap_usb v1\n");
    printf("Send: SNAP\\n (or PING / SLEEP)\n");

    if (!SNAP_LOGS_ONLY) {
        camera_self_test();
    } else {
        ESP_LOGW(TAG, "logs-only: camera self-test skipped");
    }

    if (SNAP_AUTOSNAP && !SNAP_LOGS_ONLY) {
        vTaskDelay(pdMS_TO_TICKS(1500));
        do_snap();
    }

    xTaskCreate(cmd_task, "cmd_task", 4096, NULL, 10, NULL);
}
