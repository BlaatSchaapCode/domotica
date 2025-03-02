/*
 * dsmr.h
 *
 *  Created on: 31 aug. 2023
 *      Author: andre
 */

#ifndef DSMR_H_
#define DSMR_H_

typedef struct {
	float u_volt;
	float i_ampere;
	float e_received_kilowatt;
	float e_delivered_kilowatt;
	float p_receiving;
	float p_delivering;
} phase_t;

typedef struct {
	float lo_received_kilowatthour;
	float lo_delivered_kilowatthour;
	float hi_received_kilowatthour;
	float hi_delivered_kilowatthour;
} energy_t;


typedef struct {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	char dst;
}	datetime_t;

typedef struct {
	phase_t	phases[3] ;
	energy_t energy ;
	float gas_m3 ;
	datetime_t electricity_time;
	datetime_t gas_time;
	int tariff;
}	meterstand_t;

meterstand_t dsmr_parse(char* data, size_t size);


#endif /* DSMR_H_ */
