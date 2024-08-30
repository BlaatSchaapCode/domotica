/*
 * light_switch.c
 *
 *  Created on: 19 jan. 2024
 *      Author: andre
 */

#include "light_switch.h"

#include <bshal_gpio.h>

void light_switch_set(bool value) { bshal_gpio_write_pin(1, value); }
bool light_switch_get(void) { return bshal_gpio_read_pin(1); }
bool button1_get(void) { return !bshal_gpio_read_pin(0); }
bool button2_get(void) { return !bshal_gpio_read_pin(2); }
