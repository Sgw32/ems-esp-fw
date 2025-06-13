#include "device_sm.h"
#include "ems_setup.h"
#include "pwr_button.h"
#include "ssd1306_display.h"
#include "esp_log.h"
#include "hrm.h"

static const char *TAG = "DEVICE_SM";
static device_sm_t device;
uint32_t state_counter = 0;
bool init_done = 0;

void device_sm_init() {
    ESP_LOGI(TAG, "Initializing State Machine");
    
    device.state = DEVICE_STATE_OFF;
    device.is_powered = false;
    device.pulse_measured = false;
    device.pulse_value = 0;
}

void process_state_off(void)
{
    pwr_button_handle();
    if (pwr_get_pressed_rising())
    {
        state_counter = 0;
        init_done = false;
        device.state = DEVICE_STATE_BOOT;
    }
}

void process_state_boot(void)
{
    pwr_button_handle();
    // ESP_LOGI(TAG, "Device starting...wait 1 minute");
    if (!init_done)
    {
        latch_power_on();
        device.is_powered = true;
        init_done = true;
    }
    state_counter+=1;
    if (state_counter>=60000/MAIN_TASK_LOOP_TIME_MS)
    {
        device.state = DEVICE_STATE_IDLE;
    }
    if (pwr_get_pressed_rising())
    {
        device.state = DEVICE_STATE_SHUTDOWN;
    }
}

void process_state_idle(void)
{
    pwr_button_handle();
    if (pwr_get_pressed_rising())
    {
        device.state = DEVICE_STATE_SHUTDOWN;
    }
}

void process_state_shutdown(void)
{
    ESP_LOGI(TAG, "Shutting Down...");
    latch_power_off();
    device.is_powered = false;
    device.state = DEVICE_STATE_OFF;
}

void process_device_sm() 
{
    switch (device.state) {
        case DEVICE_STATE_OFF:
            process_state_off();
            break;
        case DEVICE_STATE_BOOT:
            process_state_boot();
            break;
        case DEVICE_STATE_IDLE:
            process_state_idle();
            break;
        case DEVICE_STATE_SHUTDOWN:
            process_state_shutdown();
            break;
        default:
            break;
    }
    //update_ui(device.state, get_heart_rate(), get_hrm_connected());
}
