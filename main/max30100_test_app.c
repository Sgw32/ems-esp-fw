#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_master.h"
#include "ems_setup.h"
#include "max30100/max30100.h"

#if CONFIG_APP_MAX30100_TEST

static const char *TAG = "MAX30100_TEST";

static void log_sensor_reading(const max30100_data_t *reading)
{
    if (reading->pulse_detected) {
        ESP_LOGI(TAG, "Beat detected - BPM: %.1f | SpO2: %.1f%%",
                 reading->heart_bpm, reading->spO2);
    } else {
        ESP_LOGI(TAG, "No pulse detected - BPM: %.1f | SpO2: %.1f%%",
                 reading->heart_bpm, reading->spO2);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAX30100 sensor test application");

    i2c_init();

    max30100_config_t sensor = {};
    max30100_data_t reading = {};

    esp_err_t ret = max30100_init(&sensor, I2C_BUS_PORT,
                                   MAX30100_DEFAULT_OPERATING_MODE,
                                   MAX30100_DEFAULT_SAMPLING_RATE,
                                   MAX30100_DEFAULT_LED_PULSE_WIDTH,
                                   MAX30100_DEFAULT_IR_LED_CURRENT,
                                   MAX30100_DEFAULT_START_RED_LED_CURRENT,
                                   MAX30100_DEFAULT_MEAN_FILTER_SIZE,
                                   MAX30100_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
                                   true, false);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialise MAX30100 sensor: %s",
                 esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "MAX30100 initialised, reading values every %d ms",
             CONFIG_APP_MAX30100_TEST_SAMPLE_PERIOD_MS);

    TickType_t delay_ticks = pdMS_TO_TICKS(CONFIG_APP_MAX30100_TEST_SAMPLE_PERIOD_MS);
    if (delay_ticks == 0) {
        delay_ticks = 1;
    }

    while (true) {
        ret = max30100_update(&sensor, &reading);
        if (ret == ESP_OK) {
            log_sensor_reading(&reading);
        } else {
            ESP_LOGE(TAG, "Failed to read MAX30100 sensor: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelay(delay_ticks);
    }
}

#endif // CONFIG_APP_MAX30100_TEST
