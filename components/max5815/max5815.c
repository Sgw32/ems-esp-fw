#include "max5815.h"
#include "esp_log.h"

#define TAG "MAX5815"
#define MAX5815_CMD_WRITE_CH(n)   (0x10 | ((n) & 0x03))  // Command to write channel n

static esp_err_t max5815_write_cmd(max5815_dev_t *dev, uint8_t cmd, uint16_t value)
{
    uint8_t buf[3];
    buf[0] = cmd;
    buf[1] = (value >> 4) & 0xFF;     // MSB
    buf[2] = (value & 0x0F) << 4;     // LSB (upper 4 bits)

    return i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr, buf, 3, pdMS_TO_TICKS(100));
}

esp_err_t max5815_init(max5815_dev_t *dev, i2c_port_t port, uint8_t address)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    dev->i2c_port = port;
    dev->i2c_addr = address;
    ESP_LOGI(TAG, "Initialized MAX5815 on I2C port %d, addr 0x%02X", port, address);
    return ESP_OK;
}

esp_err_t max5815_set_channel(max5815_dev_t *dev, max5815_channel_t channel, uint16_t value)
{
    if (!dev || channel > MAX5815_CHANNEL_D || value > 0x0FFF)
        return ESP_ERR_INVALID_ARG;

    uint8_t cmd = MAX5815_CMD_WRITE_CH(channel);
    return max5815_write_cmd(dev, cmd, value);
}

esp_err_t max5815_shutdown(max5815_dev_t *dev, max5815_channel_t channel)
{
    if (!dev || channel > MAX5815_CHANNEL_D)
        return ESP_ERR_INVALID_ARG;

    // Shutdown is done by writing 0x0000 to the channel (documented behavior)
    return max5815_set_channel(dev, channel, 0);
}
