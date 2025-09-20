#include "hrm.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
#include <string.h>
/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "blehr_sens.h"
#include "gatt_svr.h"
#include "ems_setup.h"

#include "ems_common_driver/esp32/esp32_id.h"
#include "max30100/max30100.h"

static const char *tag = "HRM";

static TimerHandle_t blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

#define HRM_BLE_DEVICE_NAME_MAX_LEN   (BLE_GAP_DEVNAME_MAX_LEN + 1)

static char device_name[HRM_BLE_DEVICE_NAME_MAX_LEN] = "ems_ble";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

static void hrm_init_device_name(void);

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;

static uint8_t hrm_sensor_connected = 0;
static max30100_config_t max30100 = {};
static max30100_data_t hrm_result = {};
SemaphoreHandle_t hrm_mutex;

/**
 * Utility function to log an array of bytes.
 */
void print_bytes(const uint8_t *bytes, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        MODLOG_DFLT(INFO, "%s0x%02x", i != 0 ? ":" : "", bytes[i]);
    }
}

void print_addr(const void *addr)
{
    const uint8_t *u8p;

    u8p = addr;
    MODLOG_DFLT(INFO, "%02x:%02x:%02x:%02x:%02x:%02x",
                u8p[5], u8p[4], u8p[3], u8p[2], u8p[1], u8p[0]);
}

static void hrm_init_device_name(void)
{
    char stored_name[HRM_BLE_DEVICE_NAME_MAX_LEN] = {0};

    esp_err_t err = Esp32ReadBleName(stored_name, sizeof(stored_name));
    bool have_name = (err == ESP_OK) && (stored_name[0] != '\0');

    if (!have_name)
    {
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
        {
            ESP_LOGW(tag, "Failed to read BLE name from NVS: %s", esp_err_to_name(err));
        }

        uint32_t ffit_num = 0;
        esp_err_t num_err = Esp32LoadFFitNumber(&ffit_num);
        if (num_err != ESP_OK)
        {
            if (num_err != ESP_ERR_NVS_NOT_FOUND)
            {
                ESP_LOGW(tag, "Failed to load FFit number: %s", esp_err_to_name(num_err));
            }

            ffit_num = 0;
            esp_err_t store_err = Esp32StoreFFitNumber(ffit_num);
            if (store_err != ESP_OK)
            {
                ESP_LOGW(tag, "Failed to store default FFit number: %s", esp_err_to_name(store_err));
            }
        }

        int written = snprintf(stored_name, sizeof(stored_name), "FFit%04lu", (unsigned long)ffit_num);
        if (written < 0)
        {
            stored_name[0] = '\0';
        }
        else if ((size_t)written >= sizeof(stored_name))
        {
            stored_name[sizeof(stored_name) - 1] = '\0';
        }

        esp_err_t write_err = Esp32WriteBleName(stored_name);
        if (write_err != ESP_OK)
        {
            ESP_LOGW(tag, "Failed to store BLE name '%s': %s", stored_name, esp_err_to_name(write_err));
        }
        else
        {
            ESP_LOGI(tag, "Stored default BLE name '%s' in NVS", stored_name);
        }
    }

    if (stored_name[0] == '\0')
    {
        strncpy(stored_name, "FFit0000", sizeof(stored_name));
        stored_name[sizeof(stored_name) - 1] = '\0';
    }

    strncpy(device_name, stored_name, sizeof(device_name));
    device_name[sizeof(device_name) - 1] = '\0';

    ESP_LOGI(tag, "Using BLE device name '%s'", device_name);
}


/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void blehr_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    /*
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info)
     *     o Advertising tx power
     *     o Device name
     */
    memset(&fields, 0, sizeof(fields));

    /*
     * Advertise two flags:
     *      o Discoverability in forthcoming advertisement (general)
     *      o BLE-only (BR/EDR unsupported)
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    /*
     * Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    /* Begin advertising */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(blehr_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, blehr_gap_event, NULL);
    if (rc != 0) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static void blehr_tx_hrate_stop(void)
{
    xTimerStop( blehr_tx_timer, 1000 / portTICK_PERIOD_MS );
}

/* Reset heart rate measurement */
static void blehr_tx_hrate_reset(void)
{
    int rc;

    if (xTimerReset(blehr_tx_timer, 1000 / portTICK_PERIOD_MS ) == pdPASS) {
        rc = 0;
    } else {
        rc = 1;
    }

    assert(rc == 0);

}

/* This function simulates heart beat and notifies it to the client */
static void blehr_tx_hrate(TimerHandle_t ev)
{
    static uint8_t hrm[2];
    int rc;
    struct os_mbuf *om;

    if (!notify_state) {
        blehr_tx_hrate_stop();
        heartrate = 90;
        return;
    }

    
    if (!hrm_sensor_connected)
    {
        hrm[0] = 0x06; /* contact of a sensor */
        hrm[1] = heartrate; /* storing dummy data */
        /* Simulation of heart beats */
        heartrate++;
        if (heartrate == 160) {
            heartrate = 90;
        }
        if (hrm_mutex != NULL)
        {
            if (xSemaphoreTake(hrm_mutex, portMAX_DELAY) == pdTRUE) {
                hrm_result.heart_bpm = heartrate;
                hrm_result.pulse_detected = 1;
                xSemaphoreGive(hrm_mutex);
            }
        }
    }
    else
    {
        if (hrm_mutex != NULL)
        {
            if (xSemaphoreTake(hrm_mutex, portMAX_DELAY) == pdTRUE) {
                heartrate = hrm_result.heart_bpm;
                hrm[0] = hrm_result.pulse_detected ? 0x06 : 0x00; /* contact of a sensor */
                hrm[1] = heartrate; /* storing dummy data */
                xSemaphoreGive(hrm_mutex);
            }
        }
    }
    
    om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
    rc = ble_gatts_notify_custom(conn_handle, hrs_hrm_handle, om);

    assert(rc == 0);

    blehr_tx_hrate_reset();
}

static int blehr_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            ESP_LOGI(tag, "BLE connected, requesting MTU exchange...");
            int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
            if (rc != 0) {
                ESP_LOGW(tag, "MTU exchange request failed: %d", rc);
            }
        }
        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            blehr_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        blehr_advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; cur_notify=%d\n value handle; "
                    "val_handle=%d\n",
                    event->subscribe.cur_notify, hrs_hrm_handle);
        if (event->subscribe.attr_handle == hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_reset();
        } else if (event->subscribe.attr_handle != hrs_hrm_handle) {
            notify_state = event->subscribe.cur_notify;
            blehr_tx_hrate_stop();
        }
        ESP_LOGI("BLE_GAP_SUBSCRIBE_EVENT", "conn_handle from subscribe=%d", conn_handle);
        break;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.value);
        break;

    }

    return 0;
}

static void blehr_on_sync(void)
{
    int rc;

    rc = ble_hs_id_infer_auto(0, &blehr_addr_type);
    assert(rc == 0);

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(blehr_addr_type, addr_val, NULL);

    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");

    /* Begin advertising */
    blehr_advertise();
}

static void blehr_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

void blehr_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

void get_bpm(void* param) {
    printf("MAX30100 Test\n");
    
    while(true) 
    {
        // Lock the mutex before accessing hrm_result
        if (xSemaphoreTake(hrm_mutex, portMAX_DELAY) == pdTRUE)
        {
            // Update sensor, saving to hrm_result
            esp_err_t ret = max30100_update(&max30100, &hrm_result);
            if (ret==ESP_OK)
            {
                if (hrm_result.pulse_detected)
                {
                    printf("BEAT\n");
                    printf("BPM: %f | SpO2: %f%%\n", hrm_result.heart_bpm, hrm_result.spO2);
                }
                else
                {
                    printf("No pulse detected\n");
                }
            }
            else
            {
                printf("Error reading MAX30100: %d\n", ret);
            }
            
            // Unlock the mutex after accessing hrm_result
            xSemaphoreGive(hrm_mutex);
        }
        //Update rate: 100Hz
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

uint16_t get_heart_rate()
{
    uint16_t res = 0;
    if (hrm_mutex != NULL)
    {
        if (xSemaphoreTake(hrm_mutex, portMAX_DELAY) == pdTRUE)
        {
            res = hrm_result.heart_bpm;
            xSemaphoreGive(hrm_mutex);
        }
    }
    return res;
}

uint8_t get_hrm_connected()
{
    if (hrm_mutex != NULL)
    {
        bool pulse_detected = false;
        if (xSemaphoreTake(hrm_mutex, portMAX_DELAY) == pdTRUE)
        {
            pulse_detected = hrm_result.pulse_detected;
            xSemaphoreGive(hrm_mutex);
        }
        return pulse_detected;
    }
    return 0;
}

void init_hrm(void)
{
    int rc;
    esp_err_t ret;
    hrm_mutex = xSemaphoreCreateMutex();
    if (hrm_mutex == NULL) {
        printf("Failed to create mutex\n");
        return;
    }

    hrm_init_device_name();
    //Init sensor at I2C_NUM_0
    ret = max30100_init( &max30100, I2C_BUS_PORT,
                   MAX30100_DEFAULT_OPERATING_MODE,
                   MAX30100_DEFAULT_SAMPLING_RATE,
                   MAX30100_DEFAULT_LED_PULSE_WIDTH,
                   MAX30100_DEFAULT_IR_LED_CURRENT,
                   MAX30100_DEFAULT_START_RED_LED_CURRENT,
                   MAX30100_DEFAULT_MEAN_FILTER_SIZE,
                   MAX30100_DEFAULT_PULSE_BPM_SAMPLE_SIZE,
                   true, false );
    
    if (ret==ESP_OK)
    {
        //hrm_sensor_connected=1;
        //Start test task
        //xTaskCreatePinnedToCore(get_bpm, "Get BPM", 8192, NULL, 1, NULL, 0);
    }
    else
    {
        ESP_LOGE(tag, "MAX30100 is not connected! Using emulated data");
    }
    
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = blehr_on_sync;
    ble_hs_cfg.reset_cb = blehr_on_reset;

    /* name, period/time,  auto reload, timer ID, callback */
    blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, blehr_tx_hrate);

    rc = ems_gatt_svr_init();
    assert(rc == 0);

    if (ble_att_set_preferred_mtu(512) != 0) 
    {
        ESP_LOGE(tag, "Failed to set preferred MTU\n");
    }
    else
    {
        ESP_LOGI(tag, "Preferred MTU set to 512 bytes\n");
    }
    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(blehr_host_task);
}

