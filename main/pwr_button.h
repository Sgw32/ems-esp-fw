#ifndef PWR_BUTTON
#define PWR_BUTTON
#include <stdint.h>
#include <stdbool.h>

void pwr_button_init(void);
void pwr_button_handle(void);
bool pwr_get_pressed(void);
bool pwr_get_pressed_rising(void);
bool pwr_get_pressed_falling(void);
void latch_power_on(void);
void latch_power_off(void);

#endif