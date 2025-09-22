#ifndef GATT_SVR_H
#define GATT_SVR_H

#include <stdint.h>
#include "host/ble_gatt.h"

int ems_gatt_svr_init(void);
void ems_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
uint8_t ems_get_power_en(void);

void init_ems_ble(void);
uint16_t get_heart_rate();
uint8_t get_hrm_connected();

#endif
