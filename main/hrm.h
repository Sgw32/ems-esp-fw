#ifndef HRM_H
#define HRM_H
#include <stdint.h>
//Heart rate monitor

void init_hrm(void);
uint16_t get_heart_rate();
uint8_t get_hrm_connected();
#endif 