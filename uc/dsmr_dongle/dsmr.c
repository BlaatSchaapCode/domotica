/*
 * dsmr.c
 *
 *  Created on: 31 aug. 2023
 *      Author: andre
 */

// TODO: Check receiving/delivering are they swapped?


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define L1_VOLTAGE	"32.7.0"
#define L2_VOLTAGE	"52.7.0"
#define L3_VOLTAGE	"72.7.0"

#define L1_CURRENT	"31.7.0"
#define L2_CURRENT	"51.7.0"
#define L3_CURRENT	"71.7.0"

#define L1_POWER_DELIVERING	"21.7.0"
#define L2_POWER_DELIVERING	"41.7.0"
#define L3_POWER_DELIVERING	"61.7.0"

#define L1_POWER_RECEIVING	"22.7.0"
#define L2_POWER_RECEIVING	"42.7.0"
#define L3_POWER_RECEIVING	"62.7.0"

#define POWER_DELIVERED "1.7.0"
#define POWER_RECEIVED "2.7.0"


#define DATETIME	"1.0.0"

#define	POWER_RECEIVING_LO	"1.8.1"
#define	POWER_RECEIVING_HI	"1.8.2"

#define	POWER_DELIVERING_LO	"2.8.1"
#define	POWER_DELIVERING_HI	"2.8.2"

#define TARIFF	"96.14.0"

#define GAS	"24.2.1"

#include "dsmr.h"

meterstand_t dsmr_parse(char* data, size_t size) {
	char * entry = NULL;
	meterstand_t meterstand = {};

	entry = strstr(data, GAS);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry) {

			int count = sscanf(entry,"(%02d%02d%02d%02d%02d%02d%c",
				&meterstand.gas_time.year,
				&meterstand.gas_time.month,
				&meterstand.gas_time.day,
				&meterstand.gas_time.hour,
				&meterstand.gas_time.minute,
				&meterstand.gas_time.second,
				&meterstand.gas_time.dst);
				meterstand.gas_time.year += 2000;
				entry = strstr(entry+1 ,"(");
				if (entry) {
					meterstand.gas_m3 = strtof (entry + 1, NULL);
				}
			}
		}


	entry = strstr(data, DATETIME);
	if (entry) {
		// YYMMDDhhmmssX
		// 101209113020W
		entry = strstr(entry,"(");
		if (entry) {

			int count = sscanf(entry,"(%02d%02d%02d%02d%02d%02d%c",
			&meterstand.electricity_time.year,
			&meterstand.electricity_time.month,
			&meterstand.electricity_time.day,
			&meterstand.electricity_time.hour,
			&meterstand.electricity_time.minute,
			&meterstand.electricity_time.second,
			&meterstand.electricity_time.dst);
			meterstand.electricity_time.year += 2000;
		}
	}

	entry = strstr(data, L1_VOLTAGE);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry) {
			meterstand.phases[0].u_volt = strtof (entry + 1, NULL);
		}
	}

	entry = strstr(data, L2_VOLTAGE);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[1].u_volt = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L3_VOLTAGE);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[2].u_volt = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L1_CURRENT);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[0].i_ampere = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L2_CURRENT);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[1].i_ampere = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L3_CURRENT);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[2].i_ampere = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L1_POWER_RECEIVING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[0].e_received_kilowatt= strtof (entry + 1, NULL);
	}

	entry = strstr(data, L2_POWER_RECEIVING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[1].e_received_kilowatt= strtof (entry + 1, NULL);
	}

	entry = strstr(data, L3_POWER_RECEIVING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[2].e_received_kilowatt= strtof (entry + 1, NULL);
	}

	entry = strstr(data, L1_POWER_DELIVERING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[0].e_delivered_kilowatt = strtof (entry + 1, NULL);
	}

	entry = strstr(data, L2_POWER_DELIVERING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[1].e_delivered_kilowatt= strtof (entry + 1, NULL);
	}

	entry = strstr(data, L3_POWER_DELIVERING);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.phases[2].e_delivered_kilowatt= strtof (entry + 1, NULL);
	}

	entry = strstr(data, POWER_RECEIVING_LO);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.energy.lo_received_kilowatthour= strtof (entry + 1, NULL);
	}

	entry = strstr(data, POWER_RECEIVING_HI);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.energy.hi_received_kilowatthour= strtof (entry + 1, NULL);
	}

	entry = strstr(data, POWER_DELIVERING_LO);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
				meterstand.energy.lo_delivered_kilowatthour= strtof (entry + 1, NULL);
	}

	entry = strstr(data, POWER_DELIVERING_HI);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry)
			meterstand.energy.hi_delivered_kilowatthour= strtof (entry + 1, NULL);
	}


	entry = strstr(data, TARIFF);
	if (entry) {
		entry = strstr(entry,"(");
		if (entry) {
			meterstand.tariff=entry[4];
		}
	}

	return meterstand;
}

