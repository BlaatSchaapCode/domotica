/*
 * ds18b20.h
 *
 *  Created on: 11 sep. 2024
 *      Author: andre
 */

#ifndef DS18B20_H_
#define DS18B20_H_

#include <stdint.h>
#include <stdlib.h>

#define DS18S20_FAMILY_CODE 0x10
#define DS18B20_FAMILY_CODE 0x28
#define DS1822_FAMILY_CODE 0x22

typedef struct {
	uint64_t device_id;
	uint8_t collision_pos;
	uint8_t collision_id;
} ds18b20_t;

int ds18x20_scan_bus(ds18b20_t *ds18b20, size_t size) ;
int ds18x20_read(ds18b20_t *ds18b20, float * temperature_f) ;
int ds18x20_convert(ds18b20_t *ds18b20);
void ds18x20_init() ;



#endif /* DS18B20_H_ */
