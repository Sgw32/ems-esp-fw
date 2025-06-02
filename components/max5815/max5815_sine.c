#include "max5815_sine.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "max5815_sine";
static max5815_dev_t* sine_dev = NULL;
static TaskHandle_t sine_task_handle = NULL;
static bool sine_running = false;

// Pre-calculated sine wave lookup table
static uint16_t sine_table[SINE_SAMPLES];

static void init_sine_table(void) {
    for (int i = 0; i < SINE_SAMPLES; i++) {
        // Generate sine values scaled to DAC range
        double angle = (2.0 * M_PI * i) / SINE_SAMPLES;
        sine_table[i] = (uint16_t)((sin(angle) + 1.0) * DAC_MAX_VALUE / 2);
    }
}

static void sine_generator_task(void* pvParameters) {
    uint32_t sample_idx = 0;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / SINE_UPDATE_HZ);

    while (sine_running) {
        // Output same sine value to all channels
        for (int ch = MAX5815_CHANNEL_A; ch <= MAX5815_CHANNEL_D; ch++) {
            max5815_set_channel(sine_dev, ch, sine_table[sample_idx]);
        }

        sample_idx = (sample_idx + 1) % SINE_SAMPLES;
        vTaskDelayUntil(&last_wake_time, period);
    }

    vTaskDelete(NULL);
}

esp_err_t max5815_sine_init(max5815_dev_t *dev) {
    if (dev == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sine_dev = dev;
    init_sine_table();
    
    ESP_LOGI(TAG, "Sine wave generator initialized");
    return ESP_OK;
}

esp_err_t max5815_sine_start(void) {
    if (sine_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!sine_running) {
        sine_running = true;
        BaseType_t ret = xTaskCreate(sine_generator_task,
                                   "sine_gen",
                                   2048,
                                   NULL,
                                   5,
                                   &sine_task_handle);
        
        if (ret != pdPASS) {
            sine_running = false;
            return ESP_ERR_NO_MEM;
        }
    }
    
    return ESP_OK;
}

esp_err_t max5815_sine_stop(void) {
    if (sine_running) {
        sine_running = false;
        vTaskDelay(pdMS_TO_TICKS(10)); // Allow task to clean up
    }
    return ESP_OK;
}