#ifndef DEVICE_SM_H
#define DEVICE_SM_H
//Device State machine

#include <stdint.h>
#include <stdbool.h>
#include "ems_setup.h"

// Device structure
typedef struct {
    device_state_t state;
    bool is_powered;          // Power is on
    bool pulse_measured;      // Measurement is ok
    uint16_t pulse_value;     // Last pulse value
} device_sm_t;

void device_sm_init(void);
void process_device_sm(void);

void process_state_off(void);
void process_state_boot(void);
void process_state_idle(void);
void process_state_shutdown(void);

#endif