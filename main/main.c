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

#include "ems_setup.h"
#include "hrm.h"
#include "ssd1306_display.h"
#include "pwr_button.h"
#include "device_sm.h"
#include "ems_ble_ota.h"

static const char *TAG = "EMS main";

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
    //init_ota();
    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    
    ESP_LOGI(TAG, "Pre-setup UI");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    setup_ui();

    // Main loop
    TickType_t last_time = xTaskGetTickCount(); // Запоминаем текущее время
    while (1) {
        process_device_sm();  //Process SM
        // Ensure 20 ms loop
        vTaskDelayUntil(&last_time, pdMS_TO_TICKS(MAIN_TASK_LOOP_TIME_MS));
    }
}
