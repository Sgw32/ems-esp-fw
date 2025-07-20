#ifndef PWR_BUTTON
#define PWR_BUTTON

/** 
 * @file pwr_button.h
 * @brief Power Button Interface
 *
 * This file declares functions for handling the power button, 
 * detecting button presses, and managing power latching.
 */

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initializes the power button.
 *
 * This function sets up the power button GPIO and necessary interrupts.
 */
void pwr_button_init(void);

/**
 * @brief Handles power button events.
 *
 * This function should be called periodically to process power button states.
 */
void pwr_button_handle(void);

/**
 * @brief Checks if the power button is currently pressed.
 *
 * @return true if the button is pressed, false otherwise.
 */
bool pwr_get_pressed(void);

/**
 * @brief Detects a rising edge press event on the power button.
 *
 * @return true if a rising edge (press) is detected, false otherwise.
 */
bool pwr_get_pressed_rising(void);

/**
 * @brief Detects a falling edge release event on the power button.
 *
 * @return true if a falling edge (release) is detected, false otherwise.
 */
bool pwr_get_pressed_falling(void);

/**
 * @brief Latches the power on.
 *
 * This function ensures the device remains powered on.
 */
void latch_power_on(void);

/**
 * @brief Latches the power off.
 *
 * This function cuts off power to the device.
 */
void latch_power_off(void);

#endif /* PWR_BUTTON */
