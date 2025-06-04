#include "max5815.h"
#include <string.h>

static const char *TAG = "MAX5815";

#define MAX5815_CMD_WRITE        0x30  // Write and Load DAC
#define MAX5815_CMD_SHUTDOWN     0x20
#define MAX5815_CMD_REF_2V5      0x71
#define I2C_MASTER_TIMEOUT_MS 1000

static esp_err_t max5815_write_registers(max5815_dev_t *dev, uint8_t cmd, uint8_t msb, uint8_t lsb)
{
    uint8_t buf[3] = { cmd, msb, lsb };
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(handle, buf, sizeof(buf), true);
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(dev->i2c_port, handle, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(handle);
    return ret;
}

static esp_err_t max5815_detect(i2c_port_t i2c_port, uint8_t i2c_addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX5815 not detected at address 0x%02X", i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }
    
    ESP_LOGI(TAG, "MAX5815 detected successfully at address 0x%02X", i2c_addr);
    return ESP_OK;
}

esp_err_t max5815_init(max5815_dev_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
{
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Try to detect the device first
    esp_err_t ret = max5815_detect(i2c_port, i2c_addr);

    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;
    dev->is_initialized = false;

    // Configure CLR pin if specified
    if (dev->clr_pin != MAX5815_CLR_PIN_NONE) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dev->clr_pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        
        ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure CLR pin");
            return ret;
        }

        // Set initial CLR pin state
        ret = max5815_set_clr(dev, true);  // Initial set to true
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Add small delay between transitions

        ret = max5815_set_clr(dev, false);  // Set to false
        if (ret != ESP_OK) {
            return ret;
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Add small delay between transitions

        ret = max5815_set_clr(dev, true);  // Back to true
        if (ret != ESP_OK) {
            return ret;
        }
    }

    ret = max5815_set_internal_ref(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set internal reference voltage");
        return ret;
    }

    dev->is_initialized = true;
    ESP_LOGI(TAG, "MAX5815 initialized");

    return ESP_OK;
}

esp_err_t max5815_set_channel(max5815_dev_t *dev, max5815_channel_t channel, uint16_t value)
{
    if (!dev || !dev->is_initialized || channel > MAX5815_CHANNEL_D) {
        return ESP_ERR_INVALID_ARG;
    }

    value &= 0x0FFF;
    uint8_t cmd = MAX5815_CMD_WRITE | channel;
    uint8_t msb = value >> 4;
    uint8_t lsb = (value & 0x0F) << 4;

    return max5815_write_registers(dev, cmd, msb, lsb);
}

esp_err_t max5815_shutdown(max5815_dev_t *dev, max5815_channel_t channel)
{
    if (!dev->is_initialized || channel > MAX5815_CHANNEL_D) {
        return ESP_ERR_INVALID_ARG;
    }

    return max5815_write_registers(dev,
                                 MAX5815_CMD_SHUTDOWN | (channel << 1),
                                 0x00,
                                 0x00);
}

esp_err_t max5815_set_clr(max5815_dev_t *dev, bool enabled)
{
    if (dev->clr_pin == MAX5815_CLR_PIN_NONE) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    gpio_set_level(dev->clr_pin, enabled ? 1 : 0);
    ESP_LOGI(TAG, "CLR pin set to %s", enabled ? "enabled" : "clear");
    
    return ESP_OK;
}

esp_err_t max5815_set_internal_ref(max5815_dev_t *dev)
{
    if (!dev || !dev->is_initialized) return ESP_ERR_INVALID_STATE;
    return max5815_write_registers(dev, MAX5815_CMD_REF_2V5, 0x00, 0x00);
}
