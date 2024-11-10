/*
 * SensorManager.hpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#ifndef SENSORMANAGER_HPP_
#define SENSORMANAGER_HPP_


#include <set>
#include <mutex>
#include <map>

#include <stdint.h>

#include <sqlite3.h>
#include <nlohmann/json.hpp>



class SensorManager {
public:
	SensorManager();
	virtual ~SensorManager();

	void begin(void);

	void dongleArrived(uint32_t);
	void dongleLeft(uint32_t);

	void nodeInfoReset(uint32_t dongle_id, uint8_t node_id);
	void nodeInfoAddSensor(uint32_t dongle_id, uint8_t node_id, uint8_t sensor_id, uint8_t sensor_flags);
	void nodeInfoAddSwitch(uint32_t dongle_id, uint8_t node_id, uint8_t switch_id, uint8_t switch_flags);
	void nodeInfoPublish(uint32_t dongle_id, uint8_t node_id);


	std::string getDeviceClass(uint8_t sensor_flags);
	std::string getUnitOfMeasurement(uint8_t sensor_flags);

private:

	std::map<uint32_t, std::map<uint8_t, nlohmann::json> > mConfiguration;
	std::set<uint32_t> mDongles;
	std::mutex mMutex;
	static void sensorThread(SensorManager*);

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
