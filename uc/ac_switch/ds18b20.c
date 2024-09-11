/*
 * ds18b20.c
 *
 *  Created on: 11 sep. 2024
 *      Author: andre
 */

#include "ds18b20.h"

#include <stdbool.h>
#include <string.h>

#include "system.h"

#include <stdfix.h>
#include <stdio.h>

#include "bshal_spim.h"
#include "bshal_delay.h"
#include "bshal_i2cm.h"
#include "bshal_gpio.h"

#include "timer.h"



#define OW_ROM_MATCH    0x55
#define OW_ROM_SKIP     0xcc
#define OW_ROM_SEARCH   0xf0

#define DS18B20_TEMPERATURE_CONVERT 0x44

#define DS18X20_SCRATCHPAD_READ		0xBE
#define DS18X20_SCRATCHPAD_WRITE	0x4E

#define DS18X20_EEPROM_WRITE		0x48
#define DS18X20_EEPROM_READ			0xB8

#define DS18B20_POWER_STATUS		0xB4




#define ONEWIRE_PIN 17

void ds18x20_init() {

	bshal_gpio_cfg_out(ONEWIRE_PIN, opendrain, true);

}

int ds18x20_read_bit(uint8_t pin, bool *val) {
	bshal_gpio_write_pin(pin, false);
	bshal_delay_us(5);
	bshal_gpio_write_pin(pin, true);
//	bshal_delay_us(15);
	// Seems our delay function is a little off
//	bshal_delay_us(5);  // does not work correctly on gd32f101cb
	bshal_delay_us(3);

	*val = 1;
	int timeout = get_time_us() + 60;
	while (get_time_us() < timeout) {
		if (!bshal_gpio_read_pin(pin)) {
			*val = 0;
		}
	}
	return 0;
}

int ds18b20_send_bit(uint8_t pin, bool val) {
	if(val) {
		bshal_gpio_write_pin(pin, false);
		bshal_delay_us(5);
		bshal_gpio_write_pin(pin, true);
		bshal_delay_us(55);
	} else {
		bshal_gpio_write_pin(pin, false);
		bshal_delay_us(60);
		bshal_gpio_write_pin(pin, true);
	}
	return 0;
}


int ds18x20_write_byte(int pin, uint8_t val) {
	int result = 0;
	for (int i = 0; i < 8; i++) {
		result = ds18b20_send_bit(pin, val & (1 << i));
		if (result)
			return result;
	}
	return result;
}

int ds18b20_read_byte(int pin, uint8_t *val) {
	int result = 0;
	*val = 0;
	bool bitval;
	for (int i = 0; i < 8; i++) {
		result = ds18x20_read_bit(pin, &bitval);
		if (result)
			return result;
		*val |= bitval << i;
	}
	return result;
}

int ds18x20_reset(int pin){
	bshal_gpio_write_pin(pin, false);
	bshal_delay_us(480);
	bshal_gpio_write_pin(pin, true);
	bshal_delay_us(15);
	int timeout = 480 + get_time_us();
	bool presense = false;

	while (get_time_us() < timeout) {
		if (!bshal_gpio_read_pin(pin)) {
			presense = true;
		}
	}
	return -!presense;
}

int ds18x20_scan_bus(ds18b20_t *ds18b20, size_t size) {
	int result;
	int entry = 0;

	while (entry < size) {
		//puts("------------------");

		if (entry && !ds18b20[entry].collision_pos)
			break;
		printf("Entry %d  Colpos %d\n" , entry, ds18b20[entry].collision_pos);

		result = ds18x20_reset(ONEWIRE_PIN);
		if (result)
			return result;

		result = ds18x20_write_byte(ONEWIRE_PIN, OW_ROM_SEARCH);
		if (result)
			return result;

		for (int i = 0; i < 64; i++) {

			bool bits[2];
			ds18x20_read_bit(ONEWIRE_PIN, bits + 0);
			ds18x20_read_bit(ONEWIRE_PIN, bits + 1);
			if (bits[0] == bits[1]) {
				if (bits[0]) {
					// All devices found
					break;
				} else {
					printf("Entry %d  Colpos %d CurCol %d\n" , entry, ds18b20[entry].collision_pos, i);
					// Collision
					if (i < ds18b20[entry].collision_pos) {
						// Collision already handled
						bits[0] = ds18b20[entry].device_id & (1 << i);
						printf("Entry %d  Colpos %d CurCol %d Handled Bitval %d\n" , entry, ds18b20[entry].collision_pos, i,  bits[0]);
					} else if (i == ds18b20[entry].collision_pos) {
						// Current collision, part 2
						bits[0] = 1;
						printf("Second pass for the current collision, setting bit to %d\n" , bits[0]);
						printf("Entry %d  Colpos %d CurCol %d Pass 2  Bitval %d\n" , entry, ds18b20[entry].collision_pos, i,  bits[0]);
					} else {
						bits[0] = 0;
						printf("Entry %d  Colpos %d CurCol %d Pass 1  Bitval %d\n" , entry, ds18b20[entry].collision_pos, i,  bits[0]);
						if ((entry + 1) < size) {
							for (int j = entry + 1; j < size; j++) {
								if (!ds18b20[j].collision_pos) {
									printf("Entering collision for second pass at entry %d\n",j);
									ds18b20[j] = ds18b20[entry];
									ds18b20[j].collision_pos = i;
									ds18b20[j].collision_id++;
								break;
								}
							}
						}
					}
				}
			}
			ds18b20[entry].device_id |= (uint64_t) bits[0] << i;
			ds18b20_send_bit(ONEWIRE_PIN, bits[0]);
		}


		entry++;

	}
	for (int i = entry; i < size; i++)
		ds18b20[i].device_id = 0;
	return entry;
}

int ds18x20_convert(ds18b20_t *ds18b20) {
	int result;

	result = ds18x20_reset(ONEWIRE_PIN);
	if (result)
		return result;

	result = ds18x20_write_byte(ONEWIRE_PIN, OW_ROM_MATCH);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>8);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>16);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>24);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>32);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>40);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>48);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>56);
	result = ds18x20_write_byte(ONEWIRE_PIN, DS18B20_TEMPERATURE_CONVERT);
	bool readbit = 0;
	int begin = get_time_us();
	while(!readbit) {
		int timeout = get_time_us() - begin;
		if (timeout > 1000000) return -1; // timeout
		ds18x20_read_bit(ONEWIRE_PIN,&readbit);}
	int end = get_time_us();
//	printf("Conversion took %d ms\n",(end-begin)/1000);
	return result;
}

int ds18x20_read(ds18b20_t *ds18b20, float * temperature_f) {
	int result;
	result = ds18x20_reset(ONEWIRE_PIN);
	if (result)
		return result;

	result = ds18x20_write_byte(ONEWIRE_PIN, OW_ROM_MATCH);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>8);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>16);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>24);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>32);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>40);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>48);
	result = ds18x20_write_byte(ONEWIRE_PIN, ds18b20->device_id>>56);
	result = ds18x20_write_byte(ONEWIRE_PIN, DS18X20_SCRATCHPAD_READ);
	uint8_t scratchpad[9]={};
	for (int i = 0 ; i < sizeof(scratchpad); i++)
		result = ds18b20_read_byte(ONEWIRE_PIN, scratchpad+i);
	uint16_t *temperature=(uint16_t *)scratchpad;

	switch (ds18b20->device_id&0xFF) {
	case DS18B20_FAMILY_CODE:
	case DS1822_FAMILY_CODE:
		//DS18B20
		*temperature_f = 0.0625f * (float)(*temperature);
		break;
	case DS18S20_FAMILY_CODE:
		//DS1820
		//DS18S20
		*temperature_f = 0.5f * (float)(*temperature);
	}

	return result;
}
