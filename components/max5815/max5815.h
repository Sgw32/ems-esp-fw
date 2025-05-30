#pragma once

#include "driver/i2c.h"
#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX5815_I2C_ADDR_DEFAULT  0x48

typedef enum {
    MAX5815_CHANNEL_A = 0,
    MAX5815_CHANNEL_B,
    MAX5815_CHANNEL_C,
    MAX5815_CHANNEL_D
} max5815_channel_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
} max5815_dev_t;

esp_err_t max5815_init(max5815_dev_t *dev, i2c_port_t port, uint8_t address);
esp_err_t max5815_set_channel(max5815_dev_t *dev, max5815_channel_t channel, uint16_t value);
esp_err_t max5815_shutdown(max5815_dev_t *dev, max5815_channel_t channel);

#ifdef __cplusplus
}
#endif
