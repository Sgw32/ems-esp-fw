#ifndef SSD1306_DISPLAY
#define SSD1306_DISPLAY

/** 
 * @file ssd1306_display.h
 * @brief SSD1306 OLED Display Interface
 *
 * This file declares functions for initializing and updating the 
 * SSD1306 OLED display with device status information.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ems_setup.h"

/**
 * @brief Initializes the SSD1306 display UI.
 *
 * This function sets up the OLED display for showing device information.
 */
void setup_ui(void);

/**
 * @brief Updates the display with new device status.
 *
 * @param state Current device state.
 * @param heart_rate Current heart rate measurement in BPM.
 * @param hrm_active Indicates if the heart rate monitor is active (1 = active, 0 = inactive).
 */
void update_ui(device_state_t state, uint16_t heart_rate, uint8_t hrm_active);

/**
 * @brief Draw current EMS pulse parameters and a human readable state label.
 *
 * The parameters mirror those configured in @c main.c before invoking
 * @c emsStart(). The function fetches the current values from the EMS
 * configuration storage and renders them along with the provided state text.
 *
 * @param state_text Null-terminated label describing the EMS state
 *                   (e.g., "START", "PAUSE", "STOP").
 */
void ssd1306_display_params_and_state(const char *state_text);

#endif /* SSD1306_DISPLAY */
