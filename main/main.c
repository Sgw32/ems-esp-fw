/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <math.h>
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
#include "esp_ota_ops.h"

#include "ems_common_driver/ems.h"
#include "ems_common_driver/ems_common/ems_config.h"
#include "max5815.h"
#include "max5815_sine.h"

#include "ems_common_driver/user.h"
#include "sim/generic_ble.h"

static const char *TAG = "EMS main";

#define WCLK_FREQ_HZ    10000
#define WCLK_PERIOD_US  (1000000/WCLK_FREQ_HZ)  // 200us for 5kHz
#define LOG_INTERVAL_MS 1000  // Log every 1 second

static esp_timer_handle_t wclk_timer;
static bool wclk_state = false;

extern EmsSMTypeDef             emsSM;

uint32_t pingCnt = 0;


extern const struct    MuxSeries       muxSeries;
extern struct movingAver pulseCurrent[MUX_CHANNELS], pulseVoltage[MUX_CHANNELS];

struct ChannelLoad chanelLoad[MUX_CHANNELS];
struct ChannelLoad chanelLoadLast[MUX_CHANNELS];
uint8_t isLoadLastInit = 0;

OPT3001_t opt3001;

#ifdef __DEBUG_MODE__
struct EmsPulseMessage emsPulseMessage;
#endif

uint8_t                 data[2];
uint32_t                uid;
BleRetTypeDef           bleRet;
uint8_t                 str[256];

extern BleModuleTypeDef bleModule;
extern uint32_t countCommandError;
extern uint32_t countCommandOk;
extern SmEmsTypeDef smEms;

extern uint8_t setup_error;
extern MuxChannelTypeDef setup_channel_error;
extern uint8_t setup_old_value_error;
extern uint8_t setup_value_error;

extern struct EmsPercentCfg emsPercentCfg[2];


uint32_t                cntFlashValue;

volatile AtomosResultTypeDef    atomResultA, atomResultB;
uint32_t                        intAtom = 0;
uint32_t                        cntAtom = 0;
HardWareType                    hardware;

uint32_t                batVoltL, batVoltH;

bool                    buttonIsPush = false;
bool                    impulseMsgEnabled = true;
bool                    blinkOn = false;
bool                    isAfterStart = false;
uint32_t                startTime = 0;

uint32_t                uart1Speed = 0;
uint32_t                costumeId = 0;
uint32_t                costumeIds[16]; 
uint8_t                 costumeIdText[24]; 

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
 
static TaskHandle_t device_sm_task_handle = NULL;
static bool last_power_state = false;

static void device_sm_task(void *pvParameters) {
    // Main loop
    static uint16_t current_dac_value = 2048;  // Start at mid-range (2048 for 12-bit DAC)
    TickType_t last_time = xTaskGetTickCount();
    TickType_t last_log_time = last_time;  // Add this line

    for (max5815_channel_t channel = MAX5815_CHANNEL_A; 
            channel <= MAX5815_CHANNEL_B; 
            channel++) {
        max5815_set_channel(&dac_dev, channel, current_dac_value);
    }

    while (1) {
        //process_device_sm();

        // Only configure high voltage when power state changes
        bool current_power_state = ems_get_power_en();
        if (current_power_state != last_power_state) {
            configure_high_voltage(current_power_state);
            last_power_state = current_power_state;
        }

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
                 channel <= MAX5815_CHANNEL_B; 
                 channel++) {
                max5815_set_channel(&dac_dev, channel, current_dac_value);
            }
        }

        // Log value once per second
        if ((xTaskGetTickCount() - last_log_time) >= pdMS_TO_TICKS(LOG_INTERVAL_MS)) {
            // ESP_LOGI(TAG, "Current DAC value: %d", current_dac_value);
            last_log_time = xTaskGetTickCount();
        }
        vTaskDelayUntil(&last_time, pdMS_TO_TICKS(MAIN_TASK_LOOP_TIME_MS));
    }
}

void app_main(void)
{
    // check which partition is running
    const esp_partition_t *partition = esp_ota_get_running_partition();
    ESP_LOGI(TAG, "Running partition: %s", partition->label);

    // check if an OTA has been done, if so run diagnostics
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "An OTA update has been detected.");
            if (1) {
            ESP_LOGI(TAG,
                        "Diagnostics completed successfully! Continuing execution.");
            esp_ota_mark_app_valid_cancel_rollback();
            } else {
            ESP_LOGE(TAG,
                        "Diagnostics failed! Start rollback to the previous version.");
            esp_ota_mark_app_invalid_rollback_and_reboot();
            }
        }
    }

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
    pulseSetDACDevice(&dac_dev);

    pwr_button_init();
    device_sm_init(); 
      
    //init_hrm();
    configure_high_voltage(1);

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
    emsCfgSet(CFG_STIMUL_PULSE, 180); // Pulse depth in microseconds
    emsCfgSet(CFG_STIMUL_FREQ, 7); //Frequency in Hz of repeated pulses
    emsCfgSet(CFG_STIMUL_TIME, 60);
    emsCfgSet(CFG_STIMUL_RISE, 0);
    emsCfgSet(CFG_STIMUL_FAIL, 0);

    emsCfgSet(CFG_RELAX_PULSE, 180);
    emsCfgSet(CFG_RELAX_FREQ, 7);
    emsCfgSet(CFG_RELAX_TIME, 60);
    emsCfgSet(CFG_RELAX_RISE, 0);
    emsCfgSet(CFG_RELAX_FAIL, 0);
    emsCfgSet(CFG_RELAX_PULSE_PRESENT, 1);
    
    ESP_LOGI(TAG, "Start EMS common driver");
    emsStart();

    xTaskCreatePinnedToCore(
        device_sm_task,          // Task function
        "device_sm_task",        // Task name
        8192,                    // Stack size (adjust if needed)
        NULL,                    // Parameters
        1,        // Lowest priority
        &device_sm_task_handle,   // Task handle
        0
    );

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Adjust delay to control update rate
    }
}
