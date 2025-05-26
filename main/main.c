/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "i2c_master.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "blehr_sens.h"
#include "ems_setup.h"
#include "hrm.h"
#include "ssd1306_display.h"
#include "pwr_button.h"
#include "device_sm.h"
#include "ems_ble_ota.h"

static const char *TAG = "EMS main";

#define GPIO_PWM        8
#define GPIO_35V_ON     12 
#define GPIO_WCLK       47

#define WCLK_FREQ_HZ    10000
#define WCLK_PERIOD_US  (1000000/WCLK_FREQ_HZ)  // 200us for 5kHz

static esp_timer_handle_t wclk_timer;
static bool wclk_state = false;

// Timer callback function
static void wclk_timer_callback(void* arg)
{
    wclk_state = !wclk_state;
    gpio_set_level(GPIO_WCLK, wclk_state);
}

static void configure_high_voltage(bool enable) {
    static bool initialized = false;
    esp_timer_create_args_t timer_args = {
        .callback = wclk_timer_callback,
        .name = "wclk_timer"
    };

    // First time initialization
    if (!initialized) {
        // Configure GPIO outputs
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL<<GPIO_PWM) | (1ULL<<GPIO_35V_ON) | (1ULL<<GPIO_WCLK),
            .pull_down_en = 0,
            .pull_up_en = 0
        };
        gpio_config(&io_conf);

        // Create WCLK timer
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &wclk_timer));
        
        initialized = true;
    }

    if (enable) {
        // Set GPIO states for enable
        gpio_set_level(GPIO_PWM, 0);     // Set PWM to logic 0
        gpio_set_level(GPIO_35V_ON, 1);  // Set 35V_ON to logic 1
        
        // Start WCLK timer at 5kHz (200us period)
        esp_timer_start_periodic(wclk_timer, WCLK_PERIOD_US);
    } else {
        // Stop WCLK timer
        esp_timer_stop(wclk_timer);
        gpio_set_level(GPIO_WCLK, 0);    // Reset WCLK to 0
        
        // Disable all outputs
        gpio_set_level(GPIO_PWM, 1);     // Set PWM to logic 1
        gpio_set_level(GPIO_35V_ON, 0);  // Set 35V_ON to logic 0
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    i2c_init();
    i2c_scan();
    pwr_button_init();
    device_sm_init(); 
      
    init_hrm();
    configure_high_voltage(1);
    //init_ota();
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    
    ESP_LOGI(TAG, "Pre-setup UI");
    setup_ui();

    // Main loop
    TickType_t last_time = xTaskGetTickCount(); // Запоминаем текущее время
    while (1) {
        process_device_sm();  //Process SM
        configure_high_voltage(ems_get_power_en());
        // Ensure 20 ms loop
        vTaskDelayUntil(&last_time, pdMS_TO_TICKS(MAIN_TASK_LOOP_TIME_MS));
    }
}
