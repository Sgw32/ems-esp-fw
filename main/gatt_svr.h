#ifndef GATT_SVR_H
#define GATT_SVR_H

#include <stdint.h>

int ems_gatt_svr_init(void);
void ems_gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
uint8_t ems_get_power_en(void);

#endif