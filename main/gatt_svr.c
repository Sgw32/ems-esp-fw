#include "gatt_svr.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nimble/ble.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "max30100/max30100.h"

#include "ems_common_driver/ems.h"
#include "esp_ota_ops.h"
#include "blehr_sens.h"
#include "ems_setup.h"
#include "ems_common_driver/ems_drivers/uart.h"
#include "ems_common_driver/esp32/esp32_id.h"

static const char *TAG = "GATT_SVR";
static const char *manuf_name = "Future Fitness EMS";
static const char *model_num = "EMS ESP32";

static TimerHandle_t blehr_tx_timer;

static bool notify_state;

static uint16_t conn_handle;

#define BLE_DEVICE_NAME_MAX_LEN 32
static char device_name[BLE_DEVICE_NAME_MAX_LEN + 1] = "ems_ble";

static int blehr_gap_event(struct ble_gap_event *event, void *arg);

static uint8_t blehr_addr_type;

/* Variable to simulate heart beats */
static uint8_t heartrate = 90;

static uint8_t hrm_sensor_connected = 0;
static max30100_config_t max30100 = {};
static max30100_data_t hrm_result = {};
SemaphoreHandle_t hrm_mutex;

uint16_t hrs_hrm_handle;

uint8_t gatt_svr_chr_ota_control_val;
uint8_t gatt_svr_chr_ota_data_val[512];

uint16_t ota_control_val_handle;
uint16_t ota_data_val_handle;

const esp_partition_t *update_partition;
esp_ota_handle_t update_handle;
bool updating = false;
uint16_t num_pkgs_received = 0;
uint16_t packet_size = 0;

static uint8_t ems_power = 1;  // 0-100% power level
// Add the implementation near the bottom of the file, before ems_gatt_svr_init
uint8_t ems_get_power_en(void)
{
    return ems_power;
}

#define EMS_POWER_SERVICE_UUID        0xFF00  // Custom service UUID
#define EMS_POWER_CONTROL_UUID       0xFF01  // Custom characteristic UUID

#define EMS_UART_SERVICE_UUID        0xFFE0
#define EMS_UART_CHAR_UUID           0xFFE1
#define EMS_UART_MAX_DATA_LEN        256

uint16_t nus_tx_val_handle;
static uint8_t ems_uart_last_value[EMS_UART_MAX_DATA_LEN];
static uint16_t ems_uart_last_value_len;

static int
gatt_svr_chr_access_heart_rate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg);

static int
gatt_svr_chr_access_ems_power(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_chr_ota_control_cb(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt,
                                       void *arg);

static int gatt_svr_chr_ota_data_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt,
                                    void *arg);

static int gatt_svr_uart_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_nus_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

static int gatt_svr_nus_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg);

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
// service: OTA Service
// d6f1d96d-594c-4c53-b1c6-244a1dfde6d8
static const ble_uuid128_t gatt_svr_svc_ota_uuid =
    BLE_UUID128_INIT(0xd8, 0xe6, 0xfd, 0x1d, 0x4a, 024, 0xc6, 0xb1, 0x53, 0x4c,
                     0x4c, 0x59, 0x6d, 0xd9, 0xf1, 0xd6);

// characteristic: OTA Control
// 7ad671aa-21c0-46a4-b722-270e3ae3d830
static const ble_uuid128_t gatt_svr_chr_ota_control_uuid =
    BLE_UUID128_INIT(0x30, 0xd8, 0xe3, 0x3a, 0x0e, 0x27, 0x22, 0xb7, 0xa4, 0x46,
                     0xc0, 0x21, 0xaa, 0x71, 0xd6, 0x7a);

// characteristic: OTA Data
// 23408888-1f40-4cd8-9b89-ca8d45f8a5b0
static const ble_uuid128_t gatt_svr_chr_ota_data_uuid =
    BLE_UUID128_INIT(0xb0, 0xa5, 0xf8, 0x45, 0x8d, 0xca, 0x89, 0x9b, 0xd8, 0x4c,
                     0x40, 0x1f, 0x88, 0x88, 0x40, 0x23);

static const ble_uuid128_t nus_service_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t nus_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t nus_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(EMS_UART_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(EMS_UART_CHAR_UUID),
                .access_cb = gatt_svr_nus_rx_cb,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &nus_tx_val_handle,
            },
            { 0 }
        }
    },
    {
        /* Service: Heart-rate */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_HRS_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Heart-rate measurement */
                .uuid = BLE_UUID16_DECLARE(GATT_HRS_MEASUREMENT_UUID),
                .access_cb = gatt_svr_chr_access_heart_rate,
                .val_handle = &hrs_hrm_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            }, {
                /* Characteristic: Body sensor location */
                .uuid = BLE_UUID16_DECLARE(GATT_HRS_BODY_SENSOR_LOC_UUID),
                .access_cb = gatt_svr_chr_access_heart_rate,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },
    {
        /* Service: Device Information */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: * Manufacturer name */
                .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                /* Characteristic: Model number string */
                .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    {
        /* Service: EMS Power Control */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(EMS_POWER_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Characteristic: Power Level Control */
                .uuid = BLE_UUID16_DECLARE(EMS_POWER_CONTROL_UUID),
                .access_cb = gatt_svr_chr_access_ems_power,
                .flags = BLE_GATT_CHR_F_WRITE,
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },
    // {
    //     .type = BLE_GATT_SVC_TYPE_PRIMARY,
    //     .uuid = &nus_service_uuid.u,
    //     .characteristics = (struct ble_gatt_chr_def[]) {
    //         {
    //             .uuid = &nus_rx_uuid.u,
    //             .access_cb = gatt_svr_nus_rx_cb,
    //             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    //         },
    //         {
    //             .uuid = &nus_tx_uuid.u,
    //             .access_cb = gatt_svr_nus_tx_cb,
    //             .flags = BLE_GATT_CHR_F_NOTIFY,
    //             .val_handle = &nus_tx_val_handle,
    //         },
    //         { 0 }
    //     }
    // },
    {
        // service: OTA Service
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_ota_uuid.u,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    // characteristic: OTA control
                    .uuid = &gatt_svr_chr_ota_control_uuid.u,
                    .access_cb = gatt_svr_chr_ota_control_cb,
                    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                             BLE_GATT_CHR_F_NOTIFY,
                    .val_handle = &ota_control_val_handle,
                },
                {
                    // characteristic: OTA data
                    .uuid = &gatt_svr_chr_ota_data_uuid.u,
                    .access_cb = gatt_svr_chr_ota_data_cb,
                    .flags = BLE_GATT_CHR_F_WRITE,
                    .val_handle = &ota_data_val_handle,
                },
                {
                    0,
                }},
    },
    {
        0, /* No more services */
    },
};

static int
gatt_svr_chr_access_heart_rate(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    /* Sensor location, set to "Chest" */
    static uint8_t body_sens_loc = 0x01;
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_HRS_BODY_SENSOR_LOC_UUID) {
        rc = os_mbuf_append(ctxt->om, &body_sens_loc, sizeof(body_sens_loc));

        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_MODEL_NUMBER_UUID) {
        rc = os_mbuf_append(ctxt->om, model_num, strlen(model_num));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (uuid == GATT_MANUFACTURER_NAME_UUID) {
        rc = os_mbuf_append(ctxt->om, manuf_name, strlen(manuf_name));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int
gatt_svr_chr_access_ems_power(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == EMS_POWER_CONTROL_UUID) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            rc = os_mbuf_copydata(ctxt->om, 0, sizeof(ems_power), &ems_power);
            if (rc != 0) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            // Validate range
            if (ems_power > 1) {
                ems_power = 1;
            }
            return 0;
        }
    }

    return BLE_ATT_ERR_UNLIKELY;
}

void ems_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

static void update_ota_control(uint16_t conn_handle) {
  struct os_mbuf *om;
  esp_err_t err;

  // check which value has been received
  switch (gatt_svr_chr_ota_control_val) {
    case SVR_CHR_OTA_CONTROL_REQUEST:
      // OTA request
      ESP_LOGI(TAG, "OTA has been requested via BLE.");
      emsStop();
      // get the next free OTA partition
      update_partition = esp_ota_get_next_update_partition(NULL);
      // start the ota update
      err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES,
                          &update_handle);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)",
                 esp_err_to_name(err));
        esp_ota_abort(update_handle);
        gatt_svr_chr_ota_control_val = SVR_CHR_OTA_CONTROL_REQUEST_NAK;
      } else {
        gatt_svr_chr_ota_control_val = SVR_CHR_OTA_CONTROL_REQUEST_ACK;
        updating = true;

        // retrieve the packet size from OTA data
        packet_size =
            (gatt_svr_chr_ota_data_val[1] << 8) + gatt_svr_chr_ota_data_val[0];
        ESP_LOGI(TAG, "Packet size is: %d", packet_size);
        if (packet_size > sizeof(gatt_svr_chr_ota_data_val)) {
            ESP_LOGW(TAG, "Invalid OTA packet size: %d, clamping.", packet_size);
            packet_size = sizeof(gatt_svr_chr_ota_data_val);
        }
        num_pkgs_received = 0;
      }

      // notify the client via BLE that the OTA has been acknowledged (or not)
      om = ble_hs_mbuf_from_flat(&gatt_svr_chr_ota_control_val,
                                 sizeof(gatt_svr_chr_ota_control_val));
      ble_gattc_notify_custom(conn_handle, ota_control_val_handle, om);
      ESP_LOGI(TAG, "OTA request acknowledgement has been sent.");

      break;

    case SVR_CHR_OTA_CONTROL_DONE:

      updating = false;

      // end the OTA and start validation
      err = esp_ota_end(update_handle);
      if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
          ESP_LOGE(TAG,
                   "Image validation failed, image is corrupted!");
        } else {
          ESP_LOGE(TAG, "esp_ota_end failed (%s)!",
                   esp_err_to_name(err));
        }
      } else {
        // select the new partition for the next boot
        err = esp_ota_set_boot_partition(update_partition);
        if (err != ESP_OK) {
          ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!",
                   esp_err_to_name(err));
        }
      }

      // set the control value
      if (err != ESP_OK) {
        gatt_svr_chr_ota_control_val = SVR_CHR_OTA_CONTROL_DONE_NAK;
      } else {
        gatt_svr_chr_ota_control_val = SVR_CHR_OTA_CONTROL_DONE_ACK;
      }

      // notify the client via BLE that DONE has been acknowledged
      om = ble_hs_mbuf_from_flat(&gatt_svr_chr_ota_control_val,
                                 sizeof(gatt_svr_chr_ota_control_val));
      ble_gattc_notify_custom(conn_handle, ota_control_val_handle, om);
      ESP_LOGI(TAG, "OTA DONE acknowledgement has been sent.");

      // restart the ESP to finish the OTA
      if (err == ESP_OK) {
        ESP_LOGI(TAG, "Preparing to restart!");
        vTaskDelay(pdMS_TO_TICKS(REBOOT_DEEP_SLEEP_TIMEOUT));
        esp_restart();
      }

      break;

    default:
      break;
  }
}

static int gatt_svr_chr_ota_control_cb(uint16_t conn_handle,
                                       uint16_t attr_handle,
                                       struct ble_gatt_access_ctxt *ctxt,
                                       void *arg) {
  int rc;
  uint8_t length = sizeof(gatt_svr_chr_ota_control_val);

  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      // a client is reading the current value of ota control
      rc = os_mbuf_append(ctxt->om, &gatt_svr_chr_ota_control_val, length);
      return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
      break;

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      // a client is writing a value to ota control
      rc = gatt_svr_chr_write(ctxt->om, 1, length,
                              &gatt_svr_chr_ota_control_val, NULL);
      // update the OTA state with the new value
      update_ota_control(conn_handle);
      return rc;
      break;

    default:
      break;
  }

  // this shouldn't happen
  assert(0);
  return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_chr_ota_data_cb(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt,
                                    void *arg) {
  int rc;
  esp_err_t err;

  // store the received data into gatt_svr_chr_ota_data_val
  rc = gatt_svr_chr_write(ctxt->om, 1, sizeof(gatt_svr_chr_ota_data_val),
                          gatt_svr_chr_ota_data_val, NULL);

  // write the received packet to the partition
  if (updating) {
    ESP_LOGI(TAG, "OTA Data write received, updating = %d", updating);
    ESP_LOGI(TAG, "Packet size: %d | Packet #%d", packet_size, num_pkgs_received + 1);

    err = esp_ota_write(update_handle, (const void *)gatt_svr_chr_ota_data_val,
                        packet_size);
    ESP_LOGI(TAG, "esp_ota_write result: %s", esp_err_to_name(err));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "esp_ota_write failed (%s)!",
               esp_err_to_name(err));
    }

    if (num_pkgs_received == 0) {
        ESP_LOGI(TAG, "First 16 bytes of OTA binary:");
        ESP_LOG_BUFFER_HEX(TAG, gatt_svr_chr_ota_data_val, 16);
    }

    num_pkgs_received++;
    ESP_LOGI(TAG, "Received packet %d", num_pkgs_received);
  }

  return rc;
}

int ems_gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static int gatt_svr_uart_cb(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (ems_uart_last_value_len == 0) {
            return 0;
        }

        if (os_mbuf_append(ctxt->om, ems_uart_last_value,
                           ems_uart_last_value_len) != 0) {
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        return 0;

    case BLE_GATT_ACCESS_OP_WRITE_CHR: {
        uint8_t data[EMS_UART_MAX_DATA_LEN];
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

        if (len > sizeof(data)) {
            len = sizeof(data);
        }

        int rc = ble_hs_mbuf_to_flat(ctxt->om, data, len, NULL);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }

        if (len > 0) {
            memcpy(ems_uart_last_value, data, len);
        }
        ems_uart_last_value_len = len;

        ESP_LOGI(TAG, "BLE UART RX: Received %d bytes", len);
        ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);

        uartBLEInjectRxBytes(data, len);

        if (len > 0) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
            if (om) {
                //ble_gattc_notify_custom(conn_handle, nus_tx_val_handle, om);
                ESP_LOGI(TAG, "BLE UART TX: Notified %d bytes", len);
            }
        }

        return 0;
    }

    default:
        break;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static int gatt_svr_nus_rx_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t data[256];
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

    if (len > sizeof(data)) {
        len = sizeof(data);
    }

    int rc = ble_hs_mbuf_to_flat(ctxt->om, data, len, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    ESP_LOGI(TAG, "NUS RX: Received %d bytes", len);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, len, ESP_LOG_INFO);
    // Inject received BLE data into UART RX buffer for cmd_parser
    uartBLEInjectRxBytes(data, len);

    // Echo back
    // struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    // if (om) {
    //     ble_gattc_notify_custom(conn_handle, nus_tx_val_handle, om);
    //     //ESP_LOGI(TAG, "NUS TX: Echoed back %d bytes", len);
    // }

    return 0;
}

static int gatt_svr_nus_tx_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // This characteristic is notify-only, nothing to do here
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}   


// static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
//     switch (event->type) {
//     case BLE_GAP_EVENT_CONNECT:
//         if (event->connect.status == 0) {
// #ifdef IS_ESP32
//             uartBLESetConnHandle(event->connect.conn_handle);
// #endif
//         }
//         break;
//     case BLE_GAP_EVENT_MTU:
// #ifdef IS_ESP32
//         uartBLESetMTU(event->mtu.value);
// #endif
//         break;
//     // ...other events...
//     }
//     return 0;
// }

static void gatt_init_device_name(void)
{
    char stored_name[BLE_DEVICE_NAME_MAX_LEN + 1] = {0};
    esp_err_t err = Esp32ReadBleName(stored_name, sizeof(stored_name));
    if (err == ESP_OK && stored_name[0] != '\0') {
        snprintf(device_name, sizeof(device_name), "%s", stored_name);
        ESP_LOGI(TAG, "BLE device name loaded from NVS: %s", device_name);
        return;
    }

    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Failed to read BLE name from NVS: %s", esp_err_to_name(err));
    }

    uint32_t ffit_number = 0;
    esp_err_t ffit_err = Esp32LoadFFitNumber(&ffit_number);
    if (ffit_err == ESP_ERR_NVS_NOT_FOUND) {
        ffit_number = 0;
        esp_err_t store_err = Esp32StoreFFitNumber(ffit_number);
        if (store_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to store default FFit number: %s", esp_err_to_name(store_err));
        }
    } else if (ffit_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to load FFit number: %s", esp_err_to_name(ffit_err));
        ffit_number = 0;
        esp_err_t store_err = Esp32StoreFFitNumber(ffit_number);
        if (store_err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to store fallback FFit number: %s", esp_err_to_name(store_err));
        }
    }

    snprintf(device_name, sizeof(device_name), "FFit-%03" PRIu32, ffit_number);

    esp_err_t write_err = Esp32WriteBleName(device_name);
    if (write_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to store BLE name to NVS: %s", esp_err_to_name(write_err));
    }

    ESP_LOGI(TAG, "BLE device name set to %s", device_name);
}

/*
 * Enables advertising with parameters:
 *     o General discoverable mode
 *     o Undirected connectable mode
 */
static void ems_ble_advertise(void)
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
    int rc = 0;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed */
        MODLOG_DFLT(INFO, "connection %s; status=%d\n",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE connected, requesting MTU exchange...");
            // int rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
            uartBLESetConnHandle(event->connect.conn_handle);
            if (rc != 0) {
                ESP_LOGW(TAG, "MTU exchange request failed: %d", rc);
            }
        }
        if (event->connect.status != 0) {
            /* Connection failed; resume advertising */
            ems_ble_advertise();
        }
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);

        /* Connection terminated; resume advertising */
        ems_ble_advertise();
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "adv complete\n");
        ems_ble_advertise();
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
        uartBLESetMTU(event->mtu.value);
        break;

    }

    return 0;
}

static void ems_ble_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void ems_ble_on_sync(void)
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
    ems_ble_advertise();
}

void ems_ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
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

void init_ems_ble(void)
{
    int rc;
    esp_err_t ret;
    hrm_mutex = xSemaphoreCreateMutex();
    if (hrm_mutex == NULL) {
        printf("Failed to create mutex\n");
        return;
    }
    gatt_init_device_name();
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
        ESP_LOGE(TAG, "MAX30100 is not connected! Using emulated data");
    }
    
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = ems_ble_on_sync;
    ble_hs_cfg.reset_cb = ems_ble_on_reset;

    /* name, period/time,  auto reload, timer ID, callback */
    blehr_tx_timer = xTimerCreate("blehr_tx_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, blehr_tx_hrate);

    rc = ems_gatt_svr_init();
    assert(rc == 0);

    if (ble_att_set_preferred_mtu(512) != 0) 
    {
        ESP_LOGE(TAG, "Failed to set preferred MTU\n");
    }
    else
    {
        ESP_LOGI(TAG, "Preferred MTU set to 512 bytes\n");
    }
    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(device_name);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(ems_ble_host_task);
}

