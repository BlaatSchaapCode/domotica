/*
 * sensordata.hpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#ifndef SENSORDATA_HPP_
#define SENSORDATA_HPP_

#include <stdint.h>
#include "DeviceManager.hpp"

void sensorDataThread(DeviceManager *dm, uint32_t dongle_id, uint8_t node_id, unsigned int interval) ;



#endif /* SENSORDATA_HPP_ */
