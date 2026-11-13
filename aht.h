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
 * @file aht.h
 * @defgroup aht aht
 * @{
 *
 * ESP-IDF driver for humidty/temperature sensors AHT10/AHT15/AHT20
 *
 * Copyright (c) 2021 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __AHT_H__
#define __AHT_H__

#include <i2cdev.h>
#include <stdbool.h>

#define AHT_I2C_ADDRESS_GND 0x38 //!< Device address when ADDR pin connected to GND
#define AHT_I2C_ADDRESS_VCC 0x39 //!< Device address when ADDR pin connected to VCC

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device types
 */
typedef enum
{
    AHT_TYPE_AHT1x = 0, //!< AHT10, AHT15
    AHT_TYPE_AHT20,     //!< AHT20
} aht_type_t;

/**
 * Device modes
 *
 * @deprecated The sensor modes are not supported by the driver: it always
 * triggers a measurement in the command mode. The field is kept for the
 * source compatibility only.
 */
typedef enum
{
    AHT_MODE_NORMAL = 0, //!< Normal mode
    AHT_MODE_CYCLE,      //!< Continuous measurements mode, not supported by the driver
} aht_mode_t;

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    aht_type_t type;
    aht_mode_t mode; //!< Not used, see ::aht_mode_t
} aht_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev      Device descriptor
 * @param addr     Device I2C address
 * @param port     I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t aht_init_desc(aht_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t aht_free_desc(aht_t *dev);

/**
 * @brief Init device
 *
 * Waits for the sensor to power up, sends the calibration command and checks
 * the calibration bit. The command differs between the device types: `0xE1`
 * for AHT10/AHT15 and `0xBE` for AHT20.
 *
 * @param dev Device descriptor
 * @return
 *         - `ESP_OK` on success
 *         - `ESP_ERR_INVALID_STATE` if the sensor has not been calibrated
 *         - `ESP_ERR_TIMEOUT` if the sensor has not become idle
 */
esp_err_t aht_init(aht_t *dev);

/**
 * @brief Soft reset device
 *
 * Resets the sensor and initializes it again.
 *
 * @param dev Device descriptor
 * @return
 *         - `ESP_OK` on success
 *         - `ESP_ERR_TIMEOUT` if the sensor has not become idle
 */
esp_err_t aht_reset(aht_t *dev);

/**
 * @brief Get device status
 *
 * @param dev Device descriptor
 * @param[out] busy       Busy flag
 *                        - true: device currently measuring
 *                        - false: device is idle
 * @param[out] calibrated Calibration success flag
 *                        - true: sensor calibrated
 *                        - false: sensor not calibrated
 * @return `ESP_OK` on success
 */
esp_err_t aht_get_status(aht_t *dev, bool *busy, bool *calibrated);

/**
 * @brief Get temperature and relative humidity
 *
 * Triggers a measurement, waits for the sensor to finish it and reads the result.
 * The output values are set only when `ESP_OK` is returned. For the AHT20 the
 * frame contains a CRC-8 checksum, and a corrupted frame is rejected.
 *
 * @param dev Device descriptor
 * @param[out] temperature Temperature, degrees Celsius
 * @param[out] humidity    Relative humidity, percents
 * @return
 *         - `ESP_OK` on success
 *         - `ESP_ERR_TIMEOUT` if the sensor has not finished the measurement
 *         - `ESP_ERR_INVALID_STATE` if the sensor is not calibrated
 *         - `ESP_ERR_INVALID_RESPONSE` if the AHT20 frame has a wrong CRC
 */
esp_err_t aht_get_data(aht_t *dev, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __AHT_H__ */
