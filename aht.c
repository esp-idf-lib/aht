/*
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file aht.c
 *
 * ESP-IDF driver for humidty/temperature sensors AHT10/AHT15/AHT20
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include "aht.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#define I2C_FREQ_HZ 400000 // 400kHz

static const char *TAG = "aht";

#define CMD_CALIBRATE_1X      (0xe1)
#define CMD_CALIBRATE_20      (0xbe)
#define CMD_RESET             (0xba)
#define CMD_START_MEASUREMENT (0xac)

#define ARG_MODE_CALIBRATE (0x08)
#define ARG_MEAS_DATA      (0x33)

#define BIT_STATUS_BUSY BIT(7)
#define BIT_STATUS_CAL  BIT(3)

// The datasheet asks to wait at least 80 ms for the conversion and then to check the busy bit
#define MEASURE_DELAY_MS 80
// The sensor needs 350 ms to calibrate itself and only 10 ms for the AHT20 initialization command
#define CALIBRATE_DELAY_1X_MS 350
#define CALIBRATE_DELAY_20_MS 10
// The sensor needs 20 ms after the power-on and 40 ms in the case of the AHT20
#define POWER_ON_DELAY_1X_MS 20
#define POWER_ON_DELAY_20_MS 40
// The soft reset takes less than 20 ms
#define RESET_DELAY_MS 20
// Interval and number of the attempts to wait for the sensor to become idle
#define POLL_INTERVAL_MS 10
#define POLL_ATTEMPTS    10
// Number of the attempts to calibrate the sensor
#define INIT_ATTEMPTS 3
// CRC-8 of the AHT20 measurement frame: polynomial 0x31, initial value 0xff
#define CRC8_POLY 0x31
#define CRC8_INIT 0xff

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INIT;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++)
            crc = crc & 0x80 ? (uint8_t)((crc << 1) ^ CRC8_POLY) : (uint8_t)(crc << 1);
    }

    return crc;
}

static esp_err_t read_status_nolock(aht_t *dev, uint8_t *status)
{
    return i2c_dev_read(&dev->i2c_dev, NULL, 0, status, 1);
}

static esp_err_t wait_idle_nolock(aht_t *dev)
{
    for (int i = 0; i < POLL_ATTEMPTS; i++)
    {
        uint8_t status;
        CHECK(read_status_nolock(dev, &status));
        if (!(status & BIT_STATUS_BUSY))
            return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }

    ESP_LOGE(TAG, "Timeout waiting for the sensor to become idle");
    return ESP_ERR_TIMEOUT;
}

static esp_err_t send_cmd_nolock(aht_t *dev, uint8_t cmd0, uint8_t cmd1, uint8_t cmd2)
{
    uint8_t buf[3] = { cmd0, cmd1, cmd2 };

    return i2c_dev_write(&dev->i2c_dev, NULL, 0, buf, sizeof(buf));
}

static esp_err_t setup_nolock(aht_t *dev)
{
    CHECK(send_cmd_nolock(dev, dev->type == AHT_TYPE_AHT1x ? CMD_CALIBRATE_1X : CMD_CALIBRATE_20,
                          ARG_MODE_CALIBRATE, 0));
    vTaskDelay(pdMS_TO_TICKS(dev->type == AHT_TYPE_AHT1x ? CALIBRATE_DELAY_1X_MS : CALIBRATE_DELAY_20_MS));

    return wait_idle_nolock(dev);
}

static esp_err_t measure_nolock(aht_t *dev)
{
    CHECK(send_cmd_nolock(dev, CMD_START_MEASUREMENT, ARG_MEAS_DATA, 0));
    vTaskDelay(pdMS_TO_TICKS(MEASURE_DELAY_MS));

    return wait_idle_nolock(dev);
}

///////////////////////////////////////////////////////////////////////////////

esp_err_t aht_init_desc(aht_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    if (addr != AHT_I2C_ADDRESS_GND && addr != AHT_I2C_ADDRESS_VCC)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t aht_free_desc(aht_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t aht_init(aht_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    vTaskDelay(pdMS_TO_TICKS(dev->type == AHT_TYPE_AHT1x ? POWER_ON_DELAY_1X_MS : POWER_ON_DELAY_20_MS));

    esp_err_t err = ESP_OK;
    bool calibrated = false;
    for (int i = 0; i < INIT_ATTEMPTS; i++)
    {
        err = setup_nolock(dev);
        if (err != ESP_OK)
            break;

        uint8_t status;
        err = read_status_nolock(dev, &status);
        if (err != ESP_OK)
            break;

        calibrated = (status & BIT_STATUS_CAL) != 0;
        if (calibrated)
            break;

        ESP_LOGW(TAG, "The sensor is not calibrated, attempt %d of %d", i + 1, INIT_ATTEMPTS);
    }

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    if (err == ESP_OK && !calibrated)
    {
        ESP_LOGE(TAG, "The sensor is not calibrated");
        return ESP_ERR_INVALID_STATE;
    }

    return err;
}

esp_err_t aht_reset(aht_t *dev)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t cmd = CMD_RESET;
    esp_err_t err = i2c_dev_write(&dev->i2c_dev, NULL, 0, &cmd, sizeof(cmd));
    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));
    if (err == ESP_OK)
        err = setup_nolock(dev);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return err;
}

esp_err_t aht_get_status(aht_t *dev, bool *busy, bool *calibrated)
{
    CHECK_ARG(dev && (busy || calibrated));

    uint8_t status;
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t err = read_status_nolock(dev, &status);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    if (err != ESP_OK)
        return err;

    if (busy)
        *busy = (status & BIT_STATUS_BUSY) != 0;
    if (calibrated)
        *calibrated = (status & BIT_STATUS_CAL) != 0;

    return ESP_OK;
}

esp_err_t aht_get_data(aht_t *dev, float *temperature, float *humidity)
{
    CHECK_ARG(dev && (temperature || humidity));

    size_t size = dev->type == AHT_TYPE_AHT20 ? 7 : 6;
    uint8_t buf[7];

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    esp_err_t err = measure_nolock(dev);
    if (err == ESP_OK)
        err = i2c_dev_read(&dev->i2c_dev, NULL, 0, buf, size);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    if (err != ESP_OK)
        return err;

    if (buf[0] & BIT_STATUS_BUSY)
    {
        ESP_LOGE(TAG, "The sensor is still busy");
        return ESP_ERR_TIMEOUT;
    }

    if (!(buf[0] & BIT_STATUS_CAL))
    {
        ESP_LOGE(TAG, "The sensor is not calibrated");
        return ESP_ERR_INVALID_STATE;
    }

    if (size == 7 && crc8(buf, 6) != buf[6])
    {
        ESP_LOGE(TAG, "CRC mismatch, the frame is corrupted");
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (humidity)
    {
        uint32_t raw = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
        *humidity = (float)raw * 100 / 0x100000;
    }

    if (temperature)
    {
        uint32_t raw = ((uint32_t)(buf[3] & 0x0f) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
        *temperature = (float)raw * 200 / 0x100000 - 50;
    }

    return ESP_OK;
}
