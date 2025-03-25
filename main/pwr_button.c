#include "driver/gpio.h"
#include "esp_log.h"
#include "ems_setup.h"
#include "pwr_button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "PWR_BUTTON";

bool isPressed = false;
bool isPressedRising = false;
bool isPressedFalling = false;
uint8_t button_state_prev = 0;

void pwr_button_init() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_START_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_config_t pwr_latch_conf = {
        .pin_bit_mask = (1ULL << PIN_PWR_LATCH),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pwr_latch_conf);
    isPressed = false;
}

void pwr_button_handle() {
    if (button_state_prev!=gpio_get_level(PIN_START_BUTTON))
    {
        if ((gpio_get_level(PIN_START_BUTTON) == 1)&&(!isPressed)) {
            isPressed=true;
            isPressedRising=true;
            ESP_LOGI(TAG, "Button pressed");
        }

        if ((gpio_get_level(PIN_START_BUTTON) == 0)&&(isPressed)) {
            isPressed = false;
            isPressedFalling=true;
            ESP_LOGI(TAG, "Button unpressed");
        }
    }

    button_state_prev = gpio_get_level(PIN_START_BUTTON);
}

bool pwr_get_pressed(void)
{
    return isPressed;
}

bool pwr_get_pressed_rising(void)
{
    if (isPressedRising){
        isPressedRising = false;
        return true;
    }
    return false;
}

bool pwr_get_pressed_falling(void)
{
    if (isPressedFalling){
        isPressedFalling = false;
        return true;
    }
    return false;
}

void latch_power_on(void)
{
    gpio_set_level(PIN_PWR_LATCH, 1);
}

void latch_power_off(void)
{
    gpio_set_level(PIN_PWR_LATCH, 0);
}
