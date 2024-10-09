/*
 * sensordata.cpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#include "sensordata.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include "Device.hpp"
#include "DeviceManager.hpp"
#include "threadname.hpp"





void sensorDataThread(DeviceManager *dm, uint32_t dongle_id, uint8_t node_id, unsigned int interval) {
	char threadName[16];
	snprintf(threadName,16, "%08X_%d_Dt", dongle_id, node_id);
	setThreadName(threadName);

	std::this_thread::sleep_for(std::chrono::seconds(node_id));

	auto d = dm->getDevice(dongle_id);
	if (d) {
		printf("Requesting dongle %08X node %d\n", dongle_id, node_id);
		d->getInfo(node_id);
	}

	while (true) {
		std::this_thread::sleep_for(std::chrono::seconds(interval));
		auto d = dm->getDevice(dongle_id);
		if (d) {
//			printf("Requesting dongle %08X node %d\n", dongle_id, node_id);
//			d->getInfo(node_id);
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			d->getData(node_id);
		}






	}
}



