#pragma once

#include "driver/i2c.h"
#include "driver/gpio.h"
#include <esp_log.h>
#include <esp_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX5815_I2C_ADDR  0x1F
#define MAX5815_CLR_PIN_NONE -1

typedef enum {
    MAX5815_CHANNEL_A = 0,
    MAX5815_CHANNEL_B,
    MAX5815_CHANNEL_C,
    MAX5815_CHANNEL_D
} max5815_channel_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    int clr_pin;
    bool is_initialized;
    bool is_detected;
} max5815_dev_t;

/**
 * @brief Initialize MAX5815 device
 * @param dev Pointer to device handle structure
 * @param i2c_port I2C port number
 * @param i2c_addr I2C device address
 * @return ESP_OK on success
 */
esp_err_t max5815_init(max5815_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);

/**
 * @brief Set DAC channel value
 * @param dev Pointer to device handle structure
 * @param channel Channel to set
 * @param value 12-bit DAC value
 * @return ESP_OK on success
 */
esp_err_t max5815_set_channel(max5815_dev_t *dev, max5815_channel_t channel, uint16_t value);

/**
 * @brief Shutdown specific channel
 * @param dev Pointer to device handle structure
 * @param channel Channel to shutdown
 * @return ESP_OK on success
 */
esp_err_t max5815_shutdown(max5815_dev_t *dev, max5815_channel_t channel);

/**
 * @brief Set CLR pin state
 * @param dev Pointer to device handle structure
 * @param enabled true for normal operation, false for clear state
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if CLR pin not configured
 */
esp_err_t max5815_set_clr(max5815_dev_t *dev, bool enabled);

/**
 * @brief Set internal reference voltage
 * @param dev Pointer to device handle structure
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not supported
 */
esp_err_t max5815_set_internal_ref(max5815_dev_t *dev);

esp_err_t max5815_set_channel_async(max5815_dev_t *dev, max5815_channel_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif
