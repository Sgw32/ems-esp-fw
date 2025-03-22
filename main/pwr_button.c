#include "driver/gpio.h"
#include "esp_log.h"
#include "ems_setup.h"
#include "pwr_button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "PWR_BUTTON";
bool isPowered = false;

void button_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_START_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void button_handle() {
    if (gpio_get_level(PIN_START_BUTTON) == 1) {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (gpio_get_level(PIN_START_BUTTON) == 1) {
            isPowered = !isPowered;
            gpio_set_level(PIN_PWR_LATCH, isPowered ? 1 : 0);
            ESP_LOGI(TAG, "Power %s", isPowered ? "ON" : "OFF");
        }
        while (gpio_get_level(PIN_START_BUTTON) == 1);
    }
}
