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

#include "ems_common_driver/ems.h"
#include "ems_common_driver/ems_common/ems_config.h"
#include "max5815.h"
#include "max5815_sine.h"

static const char *TAG = "EMS main";

#define WCLK_FREQ_HZ    10000
#define WCLK_PERIOD_US  (1000000/WCLK_FREQ_HZ)  // 200us for 5kHz
#define LOG_INTERVAL_MS 1000  // Log every 1 second

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

static max5815_dev_t dac_dev = {
    .i2c_port = I2C_NUM_0,  // Use your configured I2C port
    .i2c_addr = MAX5815_I2C_ADDR,
    .clr_pin = GPIO_DAC_CLR,  
    .is_initialized = false
};

static void init_control_gpios(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_DECREASE) | (1ULL << GPIO_INCREASE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
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
    ESP_LOGI(TAG, "I2C initialized");
    init_control_gpios();
    // Initialize MAX5815 DAC
    max5815_init(&dac_dev, dac_dev.i2c_port, dac_dev.i2c_addr);
    // max5815_sine_init(&dac_dev);
    // max5815_sine_start();

    pwr_button_init();
    device_sm_init(); 
      
    init_hrm();
    configure_high_voltage(1);



    //init_ota();
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    
    ESP_LOGI(TAG, "Pre-setup UI");
    setup_ui();

    ESP_LOGI(TAG, "Initialize EMS common driver");
    ems_timer_init();
    pulseConfig();

    // #define PULSE_DEPTH_MIN_US              20
    // #define PULSE_DEPTH_MAX_US              250

    // #define PULSE_FREQ_MIN                  1
    // #define PULSE_FREQ_MAX                  100

    // #define STIMUL_RELAX_TIME_MIN_S         0
    // #define STIMUL_RELAX_TIME_MAX_S         60

    // #define STIMUL_RELAX_RISE_FAIL_MIN_MS   0
    // #define STIMUL_RELAX_RISE_FAIL_MAX_MS   5000

    // #define RELAX_PULSE_OFF                 0
    // #define RELAX_PULSE_ON                  1
    emsCfgSet(CFG_STIMUL_PULSE, 100);
    emsCfgSet(CFG_STIMUL_FREQ, 9);
    emsCfgSet(CFG_STIMUL_TIME, 5);
    emsCfgSet(CFG_STIMUL_RISE, 1000);
    emsCfgSet(CFG_STIMUL_FAIL, 1000);

    emsCfgSet(CFG_RELAX_PULSE, 100);
    emsCfgSet(CFG_RELAX_FREQ, 9);
    emsCfgSet(CFG_RELAX_TIME, 5);
    emsCfgSet(CFG_RELAX_RISE, 1000);
    emsCfgSet(CFG_RELAX_FAIL, 1000);
    
    ESP_LOGI(TAG, "Start EMS common driver");
    emsStart();

    // Main loop
    static uint16_t current_dac_value = 2048;  // Start at mid-range (2048 for 12-bit DAC)
    TickType_t last_time = xTaskGetTickCount();
    TickType_t last_log_time = last_time;  // Add this line

    while (1) {
        process_device_sm();
        configure_high_voltage(ems_get_power_en());

        // Read GPIO states (active low due to pull-up)
        bool decrease = !gpio_get_level(GPIO_DECREASE);
        bool increase = !gpio_get_level(GPIO_INCREASE);

        // Update DAC value based on button states
        if (increase && current_dac_value < DAC_MAX_VALUE) {
            current_dac_value = (current_dac_value + DAC_VALUE_STEP <= DAC_MAX_VALUE) ? 
                                current_dac_value + DAC_VALUE_STEP : DAC_MAX_VALUE;
        }
        if (decrease && current_dac_value > DAC_MIN_VALUE) {
            current_dac_value = (current_dac_value > DAC_VALUE_STEP) ? 
                                current_dac_value - DAC_VALUE_STEP : DAC_MIN_VALUE;
        }

        // Update all DAC channels if any change occurred
        if (increase || decrease) {
            for (max5815_channel_t channel = MAX5815_CHANNEL_A; 
                 channel <= MAX5815_CHANNEL_D; 
                 channel++) {
                max5815_set_channel(&dac_dev, channel, current_dac_value);
            }
        }

        // Log value once per second
        if ((xTaskGetTickCount() - last_log_time) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            ESP_LOGI(TAG, "Current DAC value: %d", current_dac_value);
            last_log_time = xTaskGetTickCount();
        }

        // Ensure 20 ms loop
        vTaskDelayUntil(&last_time, pdMS_TO_TICKS(MAIN_TASK_LOOP_TIME_MS));
    }
}
