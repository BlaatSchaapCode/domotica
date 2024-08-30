/*
 * light_switch.h
 *
 *  Created on: 19 jan. 2024
 *      Author: andre
 */

#ifndef LIGHT_SWITCH_H_
#define LIGHT_SWITCH_H_

#include <stdbool.h>

void light_switch_set(bool value);
bool light_switch_get(void);
bool button1_get(void);
bool button2_get(void);

#endif /* LIGHT_SWITCH_H_ */
