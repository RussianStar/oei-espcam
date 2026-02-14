/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */

#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

#include "driver/i2c_master.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define SCCB_FREQ               CONFIG_SCCB_CLK_FREQ  /*!< I2C master frequency*/
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

static int sccb_i2c_port;
static bool sccb_owns_i2c_port;
static i2c_master_bus_handle_t sccb_bus;
static i2c_master_dev_handle_t sccb_devs[128];

static esp_err_t sccb_get_dev(uint8_t slv_addr, i2c_master_dev_handle_t *out)
{
    if (sccb_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sccb_devs[slv_addr] == NULL) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = slv_addr,
            .scl_speed_hz = SCCB_FREQ,
        };
        esp_err_t err = i2c_master_bus_add_device(sccb_bus, &dev_cfg, &sccb_devs[slv_addr]);
        if (err != ESP_OK) {
            return err;
        }
    }
    *out = sccb_devs[slv_addr];
    return ESP_OK;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    esp_err_t ret;

    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = sccb_i2c_port,
        .sda_io_num = pin_sda,
        .scl_io_num = pin_scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&bus_cfg, &sccb_bus);
    return ret;
}

int SCCB_Use_Port(int i2c_num) { // sccb use an already initialized I2C port
    if (sccb_owns_i2c_port) {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    sccb_i2c_port = i2c_num;
    sccb_owns_i2c_port = true;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = sccb_i2c_port,
        .sda_io_num = -1,
        .scl_io_num = -1,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &sccb_bus);
}

int SCCB_Deinit(void)
{
    if (!sccb_owns_i2c_port) {
        return ESP_OK;
    }
    sccb_owns_i2c_port = false;
    for (size_t i = 0; i < sizeof(sccb_devs) / sizeof(sccb_devs[0]); ++i) {
        if (sccb_devs[i] != NULL) {
            i2c_master_bus_rm_device(sccb_devs[i]);
            sccb_devs[i] = NULL;
        }
    }
    if (sccb_bus != NULL) {
        i2c_del_master_bus(sccb_bus);
        sccb_bus = NULL;
    }
    return ESP_OK;
}

int SCCB_Probe(uint8_t slv_addr)
{
    if (sccb_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_probe(sccb_bus, slv_addr, 1000);
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return (uint8_t)-1;
    }
    ret = i2c_master_transmit_receive(dev, &reg, 1, &data, 1, 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    i2c_master_dev_handle_t dev = NULL;
    esp_err_t ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return -1;
    }
    uint8_t buf[2] = {reg, data};
    ret = i2c_master_transmit(dev, buf, sizeof(buf), 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    uint8_t data=0;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_master_dev_handle_t dev = NULL;
    ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return (uint8_t)-1;
    }
    ret = i2c_master_transmit_receive(dev, reg_u8, 2, &data, 1, 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    static uint16_t i = 0;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_master_dev_handle_t dev = NULL;
    ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return -1;
    }
    uint8_t buf[3] = {reg_u8[0], reg_u8[1], data};
    ret = i2c_master_transmit(dev, buf, sizeof(buf), 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%02x %d fail\n", reg, data, i++);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    uint16_t data = 0;
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    i2c_master_dev_handle_t dev = NULL;
    ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return (uint16_t)-1;
    }
    ret = i2c_master_transmit_receive(dev, reg_u8, 2, (uint8_t *)&data, 2, 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    esp_err_t ret = ESP_FAIL;
    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;
    uint16_t data_htons = LITTLETOBIG(data);
    uint8_t *data_u8 = (uint8_t *)&data_htons;
    i2c_master_dev_handle_t dev = NULL;
    ret = sccb_get_dev(slv_addr, &dev);
    if (ret != ESP_OK) {
        return -1;
    }
    uint8_t buf[4] = {reg_u8[0], reg_u8[1], data_u8[0], data_u8[1]};
    ret = i2c_master_transmit(dev, buf, sizeof(buf), 1000);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "W [%04x]=%04x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}
