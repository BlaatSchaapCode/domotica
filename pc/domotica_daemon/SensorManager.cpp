/*
 * SensorManager.cpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#include "SensorManager.hpp"

#include <stdlib.h>
#include <stdio.h>

#include "sensor_protocol.h"
#include "mqtt.hpp"
extern mqqt_test *gp_mqtt;

#include <thread>

#include "DeviceManager.hpp"

extern DeviceManager *p_dm;
void sensorDataThread(DeviceManager *dm, uint32_t dongle_id, uint8_t node_id,
		unsigned int interval);

// It seems Domoticz does not support publishing the whole device at once
// https://www.home-assistant.io/integrations/mqtt/#device-discovery-payload
// where it shows publishing a whole device with components

// For Domoticz we need to publish each component separately.
// Potentially include the device with the same id???

// But it has been tested with home assistant

SensorManager::SensorManager() {
	printf("Using SQLite3 version %s\n", sqlite3_libversion());

	int rc;
	rc = sqlite3_open("database.db", &mDb);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(mDb));
		sqlite3_close(mDb);
		mDb = nullptr;
		return;
	} else {
		fprintf(stdout, "Database opened successfully: %s\n",
				sqlite3_errmsg(mDb));
	}

	char *errmsg;
	rc = sqlite3_exec(mDb, mCreateTables, NULL, 0, &errmsg);
	if (rc != SQLITE_OK) {
		fprintf(stdout, "Failed to create tables:  %s\n", sqlite3_errmsg(mDb));
		fprintf(stdout, "Additional error message: %s\n", errmsg);
	} else {
		fprintf(stdout, "Tables are ok:  %s\n", sqlite3_errmsg(mDb));
	}
}

void SensorManager::begin(void) {
	std::thread(sensorThread, this).detach();
}

SensorManager::~SensorManager() {
	// TODO Auto-generated destructor stub
}

void SensorManager::sensorThread(SensorManager *_this) {
	sqlite3_stmt *res;
	int rc;
	while (true) {
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		if (true) {
			const std::lock_guard<std::mutex> lock(_this->mMutex);
			for (auto dongle_id : _this->mDongles) {

				const char *sqlSelect =
						"SELECT node_id, query_interval, last_queried, status FROM nodes WHERE dongle_id = ?";
				rc = sqlite3_prepare_v2(_this->mDb, sqlSelect, -1, &res, 0);
				if (rc == SQLITE_OK) {
					sqlite3_bind_int64(res, 1, dongle_id);
				} else {
					fprintf(stderr, "Failed to execute statement: %s\n",
							sqlite3_errmsg(_this->mDb));
				}
				rc = sqlite3_step(res);
				while (rc == SQLITE_ROW) {
					int node_id = sqlite3_column_int(res, 0);
					int query_interval = sqlite3_column_int(res, 1);
					int last_queried = sqlite3_column_int(res, 2);
					int status = sqlite3_column_int(res, 3);

					time_t next_query = query_interval + last_queried;
					if (time(nullptr) > next_query) {
						printf(
								"Time has reached to query dongle %08X node %02X with status %02X\n",
								dongle_id, node_id, status);
						auto d = p_dm->getDevice(dongle_id);
						if (d) {

							d->getInfo(node_id); // testint

							d->getData(node_id);

							// TODO update
							const char *sqlUpdate =
									"UPDATE nodes SET last_queried=unixepoch(), last_seen=unixepoch() WHERE dongle_id=? AND node_id=?";
							rc = sqlite3_prepare_v2(_this->mDb, sqlUpdate, -1,
									&res, 0);

							if (rc == SQLITE_OK) {
								sqlite3_bind_int64(res, 1, dongle_id);
								sqlite3_bind_int64(res, 2, node_id);
								rc = sqlite3_step(res);

								if (rc == SQLITE_DONE) {
									puts("Node has been updated");
								}
							} else {
								fprintf(stderr,
										"Failed to execute statement: %s\n",
										sqlite3_errmsg(_this->mDb));
							}
						} else {
							puts("dongle offline????");
						}
					}
					rc = sqlite3_step(res);

					// if send queue works as intended, this sleep won't be needed
					//std::this_thread::sleep_for(std::chrono::milliseconds(500));
				}
				sqlite3_finalize(res);
			}
		}
	}
}
void SensorManager::dongleArrived(uint32_t dongle_id) {
	sqlite3_stmt *res;
	int rc;
	const char *sqlSelect = "SELECT * FROM dongles WHERE dongle_id = ?";
	rc = sqlite3_prepare_v2(mDb, sqlSelect, -1, &res, 0);

	if (rc == SQLITE_OK) {
		sqlite3_bind_int64(res, 1, dongle_id);
	} else {
		fprintf(stderr, "Failed to execute statement: %s\n",
				sqlite3_errmsg(mDb));
	}

	int step = sqlite3_step(res);

	if (step == SQLITE_ROW) {

		printf("Dongle %8X has been found in the database \n",
				sqlite3_column_int(res, 0));
		const std::lock_guard<std::mutex> lock(mMutex);
		mDongles.insert(dongle_id);
		printf("Dongle %08X is now online\n", dongle_id);

	} else {
		//puts ("This Dongle has not been found in the database");
		printf("Dongle %8X has *NOT* been found in the database \n", dongle_id);

	}

	sqlite3_finalize(res);

	if (step != SQLITE_ROW)
		return;
	///
	const char *sqlUpdate =
			"UPDATE dongles SET last_seen=unixepoch() WHERE dongle_id=?";
	rc = sqlite3_prepare_v2(mDb, sqlUpdate, -1, &res, 0);

	if (rc == SQLITE_OK) {
		sqlite3_bind_int64(res, 1, dongle_id);

		step = sqlite3_step(res);

		if (step == SQLITE_DONE) {

			printf("Dongle Last Seen has been updated\n");

		} else {
			printf("Dongle Last Seen has *NOT* been updated\n");
		}

		sqlite3_finalize(res);
	} else {
		fprintf(stderr, "Failed to execute statement: %s\n",
				sqlite3_errmsg(mDb));
	}

}

void SensorManager::dongleLeft(uint32_t dongle_id) {
	const std::lock_guard<std::mutex> lock(mMutex);
	mDongles.erase(dongle_id);
	printf("Dongle %08X is now offline\n");
}

void SensorManager::nodeInfoReset(uint32_t dongle_id, uint8_t node_id) {
	mConfiguration[dongle_id][node_id].clear();

	char state_topic[256];
	char command_topic[256];

	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/device/%08X/%02X/state", dongle_id, node_id);
	snprintf(command_topic, sizeof(command_topic),
			"homeassistant/device/%08X/%02X/command", dongle_id, node_id);

	mConfiguration[dongle_id][node_id]["state_topic"] = state_topic;
	mConfiguration[dongle_id][node_id]["command_topic"] = command_topic;

	mConfiguration[dongle_id][node_id]["origin"]["name"] =
			"BlaatSchaap Domotica";
	mConfiguration[dongle_id][node_id]["origin"]["sw_version"] = "dev";
	mConfiguration[dongle_id][node_id]["origin"]["url"] =
			"https://www.blaatschaap.be";

	char identifiers[256];
	snprintf(identifiers, sizeof(identifiers), "%08X_%02X", dongle_id, node_id);

//	mConfiguration[dongle_id][node_id]["device"]	["configuration_url"];
//	mConfiguration[dongle_id][node_id]["device"]	["connections"];
	mConfiguration[dongle_id][node_id]["device"]["identifiers"] = identifiers;
	mConfiguration[dongle_id][node_id]["device"]["name"] = "AC Switch";
	mConfiguration[dongle_id][node_id]["device"]["manufacturer"] =
			"BlaatSchaap";
//	mConfiguration[dongle_id][node_id]["device"]	["model"];
//	mConfiguration[dongle_id][node_id]["device"]	["model_id"];
//	mConfiguration[dongle_id][node_id]["device"]	["hw_version"];
//	mConfiguration[dongle_id][node_id]["device"]	["sw_version"];
//	mConfiguration[dongle_id][node_id]["device"]	["suggested_area"];
//	mConfiguration[dongle_id][node_id]["device"]	["serial_number"];

}

std::string SensorManager::getDeviceClass(uint8_t sensor_flags) {

	switch (sensor_flags) {
	case (1 << bsprot_sensor_temperature):
		return "temperature";

	case (1 << bsprot_sensor_illuminance):
		return "illuminance";
	}
	return "unknown";
}

std::string SensorManager::getDeviceClass2(uint8_t sensor_flags) {

	switch (sensor_flags) {
	case (bsprot_sensor_temperature):
		return "temperature";

	case (bsprot_sensor_illuminance):
		return "illuminance";
	}
	return "unknown";
}

std::string SensorManager::getUnitOfMeasurement(uint8_t sensor_flags) {
	switch (sensor_flags) {
	case (1 << bsprot_sensor_temperature):

		return "°C";

	case (1 << bsprot_sensor_illuminance):

		return "lux";

	}
	return "unknown";
}

void SensorManager::nodeInfoAddSensor(uint32_t dongle_id, uint8_t node_id,
		uint8_t sensor_id, uint8_t sensor_flags) {
//	char unique_id[256];
//
//	for (int sensor_flag = 0x01; sensor_flag < 0x100; sensor_flag <<= 1) {
//		if (sensor_flag & sensor_flags) {
//			snprintf(unique_id, sizeof(unique_id), "%08X_%02X_%02X_%02X",
//					dongle_id, node_id, sensor_id, sensor_flag);
//
//			char value_template[256];
//			snprintf(value_template, sizeof(value_template), "{{value_json.%s_%02x}}",
//					getDeviceClass(sensor_flag).c_str(), sensor_id);
//
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["platform"] =
//					"sensor";
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["device_class"] = getDeviceClass(sensor_flag);
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["unit_of_measurement"] = getUnitOfMeasurement(sensor_flag);
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["value_template"] = value_template;
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["unique_id"] = unique_id;
//		}
//	}

	nlohmann::json json;
	char state_topic[256];
	char command_topic[256];
	char config_topic[256];
	char identifiers[256];
	char unique_id[256];

	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/device/%08X/%02X/state", dongle_id, node_id);
	snprintf(command_topic, sizeof(command_topic),
			"homeassistant/device/%08X/%02X/command", dongle_id, node_id);
	snprintf(config_topic, sizeof(config_topic),
			"homeassistant/sensor/%08X/%02X/config", dongle_id, node_id);
	snprintf(identifiers, sizeof(identifiers), "%08X_%02X", dongle_id, node_id);

	json["state_topic"] = state_topic;
	json["command_topic"] = command_topic;

	json["origin"]["name"] = "BlaatSchaap Domotica";
	json["origin"]["sw_version"] = "dev";
	json["origin"]["url"] = "https://www.blaatschaap.be";

//	json["device"]	["configuration_url"];
//	json["device"]	["connections"];
	json["device"]["identifiers"] = identifiers;
	json["device"]["name"] = "AC Switch";
	json["device"]["manufacturer"] = "BlaatSchaap";
//	json["device"]	["model"];
//	json["device"]	["model_id"];
//	json["device"]	["hw_version"];
//	json["device"]	["sw_version"];
//	json["device"]	["suggested_area"];
//	json["device"]	["serial_number"];

	char value_template[256];
	for (int sensor_flag = 0x01; sensor_flag < 0x100; sensor_flag <<= 1) {
		if (sensor_flag & sensor_flags) {
			snprintf(unique_id, sizeof(unique_id), "%08X_%02X_%02X_%02X",
					dongle_id, node_id, sensor_id, sensor_flag);

			snprintf(value_template, sizeof(value_template),
					"{{value_json.%s_%02x}}",
					getDeviceClass(sensor_flag).c_str(), sensor_id);

			json["device_class"] = getDeviceClass(sensor_flag);
			json["unit_of_measurement"] = getUnitOfMeasurement(sensor_flag);
			json["value_template"] = value_template;
			json["unique_id"] = unique_id;

			int mid;
			std::string test_std = json.dump(4); // for debug
			const char *test_cstr = test_std.c_str();
			gp_mqtt->publish(&mid, config_topic, strlen(test_cstr),
					(void*) test_cstr);
		}
	}

}
void SensorManager::nodeInfoAddSwitch(uint32_t dongle_id, uint8_t node_id,
		uint8_t switch_id, uint8_t switch_flags) {

//	char unique_id[256];
//
//	for (int switch_flag = 0x01; switch_flag < 0x100; switch_flag <<= 1) {
//		if (switch_flag &  switch_flags) {
//			snprintf(unique_id, sizeof(unique_id), "%08X_%02X_%02X_%02X",
//					dongle_id, node_id, switch_id, switch_flag);
//
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["device_class"] = "outlet";
//
//// TODO, Research value templated for this one
////			mConfiguration[dongle_id][node_id]["components"][unique_id]["payload_on"] = 1;
////			mConfiguration[dongle_id][node_id]["components"][unique_id]["payload_off"] = 0;
////			mConfiguration[dongle_id][node_id]["components"][unique_id]["state_on"] = 1;
////			mConfiguration[dongle_id][node_id]["components"][unique_id]["state_off"] = 0;
//
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["platform"] = "switch";
//
//			mConfiguration[dongle_id][node_id]["components"][unique_id]["unique_id"] = unique_id;
//		}
//	}

	nlohmann::json json;
	char state_topic[256];
	char command_topic[256];
	char config_topic[256];
	char identifiers[256];
	char unique_id[256];

	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/device/%08X/%02X/state", dongle_id, node_id);
	snprintf(command_topic, sizeof(command_topic),
			"homeassistant/device/%08X/%02X/command", dongle_id, node_id);
	snprintf(config_topic, sizeof(config_topic),
			"homeassistant/switch/%08X/%02X/config", dongle_id, node_id);
	snprintf(identifiers, sizeof(identifiers), "%08X_%02X", dongle_id, node_id);

	json["state_topic"] = state_topic;
	json["command_topic"] = command_topic;

	json["origin"]["name"] = "BlaatSchaap Domotica";
	json["origin"]["sw_version"] = "dev";
	json["origin"]["url"] = "https://www.blaatschaap.be";

//	json["device"]	["configuration_url"];
//	json["device"]	["connections"];
	json["device"]["identifiers"] = identifiers;
	json["device"]["name"] = "AC Switch";
	json["device"]["manufacturer"] = "BlaatSchaap";
//	json["device"]	["model"];
//	json["device"]	["model_id"];
//	json["device"]	["hw_version"];
//	json["device"]	["sw_version"];
//	json["device"]	["suggested_area"];
//	json["device"]	["serial_number"];

	char value_template[256];
	snprintf(value_template, sizeof(value_template), "{{value_json.%s_%02x}}",
			"outlet", switch_id);

	json["device_class"] = "outlet";
	json["value_template"] = value_template;
//	json["unique_id"];

	int mid;
	std::string test_std = json.dump(4); // for debug
	const char *test_cstr = test_std.c_str();
	gp_mqtt->publish(&mid, config_topic, strlen(test_cstr), (void*) test_cstr);

}

void SensorManager::nodeInfoPublish(uint32_t dongle_id, uint8_t node_id) {
//	// TODO
//	char config_topic[256];
//		snprintf(config_topic, sizeof(config_topic),
//				"homeassistant/device/%08X/%02X/config", dongle_id, node_id);
//// 		int publish(int *mid, const char *topic, int payloadlen=0, const void *payload=NULL, int qos=0, bool retain=false);
//		int mid;
//		std::string test_std = mConfiguration[dongle_id][node_id].dump(4); // for debug
//		const char * test_cstr = test_std.c_str();
//		gp_mqtt->publish(&mid, config_topic, strlen(test_cstr),  (void*)test_cstr);

}

void SensorManager::nodeValueReset(uint32_t dongle_id, uint8_t node_id) {
	mValue[dongle_id][node_id].clear();
}
void SensorManager::nodeValueAddSensor(uint32_t dongle_id, uint8_t node_id,
		uint8_t sensor_id, uint8_t sensor_flags, float value) {
//	// temporary
//	nodeValueReset(dongle_id, node_id);

	// "value_template": "{{value_json.temperature_03}}"
	char buffer[128];
	snprintf(buffer, sizeof(buffer), "%s_%02X",
			getDeviceClass2(sensor_flags).c_str(), sensor_id);
	mValue[dongle_id][node_id][buffer] = value;

//	// temporary
//	nodeValuePublish(dongle_id, node_id);

}
void SensorManager::nodeValueAddSwitch(uint32_t dongle_id, uint8_t node_id,
		uint8_t switch_id, uint8_t switch_flags, bool value) {
	char buffer[128];
	snprintf(buffer, sizeof(buffer), "%s_%02X", "outlet", switch_id);
	mValue[dongle_id][node_id][buffer] = value;

}
void SensorManager::nodeValuePublish(uint32_t dongle_id, uint8_t node_id) {
	// homeassistant/device/D0226E5D/06/state
	mValue[dongle_id][node_id];

	char state_topic[256];
	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/device/%08X/%02X/state", dongle_id, node_id);

	int mid;
	std::string test_std = mValue[dongle_id][node_id].dump(4);
	const char *test_cstr = test_std.c_str();
	gp_mqtt->publish(&mid, state_topic, strlen(test_cstr), (void*) test_cstr);
}
