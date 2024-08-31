/*
 * Sensor.hpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#ifndef SENSOR_HPP_
#define SENSOR_HPP_


#include <stdint.h>
#include <thread>
#include <string>


class Sensor {
public:
	Sensor();
	virtual ~Sensor();

private:
	uint32_t m_dongle_id;
	uint8_t m_node_id;
	std::string m_room;
	std::string m_name;


};

#endif /* SENSOR_HPP_ */
