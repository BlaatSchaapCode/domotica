/*
 * SensorManager.hpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#ifndef SENSORMANAGER_HPP_
#define SENSORMANAGER_HPP_

#include "Sensor.hpp"

#include <vector>

class SensorManager {
public:
	SensorManager();
	virtual ~SensorManager();
private:
	std::vector<Sensor> m_sensors;
};

#endif /* SENSORMANAGER_HPP_ */
