#ifndef SSD1306_DISPLAY
#define SSD1306_DISPLAY
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ems_setup.h"

void setup_ui();
void update_ui(device_state_t state, uint16_t heart_rate, uint8_t hrm_active);

#endif