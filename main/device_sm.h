#ifndef DEVICE_SM_H
#define DEVICE_SM_H

/** 
 * @file device_sm.h
 * @brief Device State Machine Header
 *
 * This file defines the state machine structure and related functions 
 * for managing the device states.
 */

#include <stdint.h>
#include <stdbool.h>
#include "ems_setup.h"

/**
 * @brief Device state machine structure.
 * 
 * This structure holds the state of the device, including power status,
 * pulse measurement status, and the last recorded pulse value.
 */
typedef struct {
    device_state_t state;    /**< Current state of the device */
    bool is_powered;         /**< Indicates if the device is powered on */
    bool pulse_measured;     /**< Indicates if a pulse measurement was successful */
    uint16_t pulse_value;    /**< Last recorded pulse value */
} device_sm_t;

/**
 * @brief Initializes the device state machine.
 *
 * This function sets up the initial state and parameters for the device.
 */
void device_sm_init(void);

/**
 * @brief Processes the device state machine.
 *
 * This function is called periodically to update the device state based 
 * on conditions and events.
 */
void process_device_sm(void);

/**
 * @brief Handles the OFF state of the device.
 *
 * This function is executed when the device is in the OFF state.
 */
void process_state_off(void);

/**
 * @brief Handles the BOOT state of the device.
 *
 * This function is executed when the device is in the BOOT state.
 */
void process_state_boot(void);

/**
 * @brief Handles the IDLE state of the device.
 *
 * This function is executed when the device is in the IDLE state.
 */
void process_state_idle(void);

/**
 * @brief Handles the SHUTDOWN state of the device.
 *
 * This function is executed when the device is transitioning to shutdown.
 */
void process_state_shutdown(void);

#endif /* DEVICE_SM_H */
