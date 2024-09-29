/*
 * SensorManager.hpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#ifndef SENSORMANAGER_HPP_
#define SENSORMANAGER_HPP_


#include <vector>

#include <sqlite3.h>

#include <stdint.h>

class SensorManager {
public:
	SensorManager();
	virtual ~SensorManager();

	void dongleArrived(uint32_t);
	void dongleLeft(uint32_t);

	void nodeInfoReset(uint32_t dongle_id, uint8_t node_id);
	void nodeInfoAddSensor(uint32_t dongle_id, uint8_t node_id, uint8_t sensor_id, uint8_t sensor_flags);
	void nodeInfoAddSwitch(uint32_t dongle_id, uint8_t node_id, uint8_t switch_id, uint8_t switch_flags);
	void nodeInfoPublish(uint32_t dongle_id, uint8_t node_id);

private:
	sqlite3 *mDb;

	const char * mCreateTables= ""
			"CREATE TABLE IF NOT EXISTS dongles ( "
	"dongle_id INTEGER, "
	"status INTEGER, "
	"last_seen INTEGER, "
	"last_queried INTEGER, "
	"PRIMARY KEY (dongle_id) "
");"


"CREATE TABLE IF NOT EXISTS nodes ( "
	"dongle_id INTEGER, "
	"node_id INTEGER, "
	"status INTEGER, "
	"last_seen INTEGER, "
	"last_queried INTEGER, "
	"query_interval INTEGER, "
	"PRIMARY KEY (dongle_id, node_id), "
	"FOREIGN KEY (dongle_id) "
		"REFERENCES dongles (dongle_id) "
		"ON DELETE CASCADE "
		"ON UPDATE NO ACTION "
");"


"CREATE TABLE IF NOT EXISTS sensors ( "
	"dongle_id INTEGER, "
	"node_id INTEGER, "
	"sensor_id INTEGER, "
	"sensor_type INTEGER, "
	"PRIMARY KEY (dongle_id, node_id, sensor_id), "
	"FOREIGN KEY (dongle_id, node_id) "
		"REFERENCES nodes (dongle_id, node_id) "
		"ON DELETE CASCADE "
		"ON UPDATE NO ACTION "
");"
			;


};

#endif /* SENSORMANAGER_HPP_ */
