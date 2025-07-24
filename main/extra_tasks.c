#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "pwr_button.h"
#include "device_sm.h"
#include "ems_common_driver/ems.h"

static const char *TAG = "EXTRA_TASKS";

#define MAIN_TASK_DELAY_MS 50
#define CMD_TASK_DELAY_MS  50
#define BT_TASK_DELAY_MS  250

static void handle_button_actions(void)
{
    pwr_button_handle();

    if (pwr_get_pressed_rising()) {
        ESP_LOGI(TAG, "Button pressed");
        emsStart();
    }
    if (pwr_get_pressed_falling()) {
        ESP_LOGI(TAG, "Button released");
        emsPause();
    }
}

void main_task(void *param)
{
    while (1) {
        handle_button_actions();
        vTaskDelay(pdMS_TO_TICKS(MAIN_TASK_DELAY_MS));
    }
}

void cmd_proc_task(void *param)
{
    while (1) {
        // Placeholder for command processing logic
        vTaskDelay(pdMS_TO_TICKS(CMD_TASK_DELAY_MS));
    }
}

void bt_task(void *param)
{
    while (1) {
        // Placeholder for bluetooth handling logic
        vTaskDelay(pdMS_TO_TICKS(BT_TASK_DELAY_MS));
    }
}

