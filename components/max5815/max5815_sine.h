#pragma once

#include "max5815.h"
#include <esp_err.h>

// Configure sine wave parameters
#define SINE_SAMPLES       256     // Number of samples per period
#define SINE_UPDATE_HZ    1000     // Update frequency
#define DAC_MAX_VALUE    4095     // 12-bit DAC

esp_err_t max5815_sine_init(max5815_dev_t *dev);
esp_err_t max5815_sine_start(void);
esp_err_t max5815_sine_stop(void);