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


"CREATE TABLE IF NOT EXISTS sensors ( "
	"dongle_id INTEGER, "
	"sensor_id INTEGER, "
	"status INTEGER, "
	"last_seen INTEGER, "
	"last_queried INTEGER, "
	"query_interval INTEGER, "
	"PRIMARY KEY (dongle_id, sensor_id), "
	"FOREIGN KEY (dongle_id) "
		"REFERENCES dongles (dongle_id) "
		"ON DELETE CASCADE "
		"ON UPDATE NO ACTION "
");";


};

#endif /* SENSORMANAGER_HPP_ */
