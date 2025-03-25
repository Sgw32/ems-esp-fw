#ifndef I2C_MASTER
#define I2C_MASTER

/** 
 * @file i2c_master.h
 * @brief I2C Master Interface
 *
 * This file declares functions for initializing and scanning the I2C bus.
 */

#include "driver/i2c_master.h"

/**
 * @brief Initializes the I2C master interface.
 *
 * This function configures the I2C master for communication with peripheral devices.
 */
void i2c_init(void);

/**
 * @brief Scans the I2C bus for connected devices.
 *
 * This function detects and lists devices connected to the I2C bus.
 */
void i2c_scan(void);

#endif /* I2C_MASTER */
