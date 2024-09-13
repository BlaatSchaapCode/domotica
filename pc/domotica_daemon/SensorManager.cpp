/*
 * SensorManager.cpp
 *
 *  Created on: 31 aug. 2024
 *      Author: andre
 */

#include "SensorManager.hpp"

#include <stdlib.h>
#include <stdio.h>


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
		fprintf(stdout, "Database opened successfully: %s\n", sqlite3_errmsg(mDb));
	}

	char * errmsg;
	rc = sqlite3_exec(mDb, mCreateTables, NULL, 0, &errmsg);
	if (rc != SQLITE_OK ) {
		fprintf(stdout, "Failed to create tables:  %s\n", sqlite3_errmsg(mDb));
		fprintf(stdout, "Additional error message: %s\n", errmsg);
	} else {
		fprintf(stdout, "Tables are ok:  %s\n", sqlite3_errmsg(mDb));
	}




}

SensorManager::~SensorManager() {
	// TODO Auto-generated destructor stub
}

void SensorManager::dongleArrived(uint32_t dongle_id){
	sqlite3_stmt *res;
	int rc;
	 const char *sqlSelect = "SELECT * FROM dongles WHERE dongle_id = ?";
	    rc = sqlite3_prepare_v2(mDb, sqlSelect, -1, &res, 0);

	    if (rc == SQLITE_OK) {
	        sqlite3_bind_int64(res, 1, dongle_id);
	    } else {
	        fprintf(stderr, "Failed to execute statement: %s\n", sqlite3_errmsg(mDb));
	    }

	    int step = sqlite3_step(res);

	    if (step == SQLITE_ROW) {

	        printf("Dongle %8X has been found in the database \n", sqlite3_column_int(res, 0));

	    } else {
	    	puts ("This Dongle has not been found in the database");
	    }

	    sqlite3_finalize(res);


	    ///
		 const char *sqlUpdate = "UPDATE dongles SET last_seen=unixepoch() WHERE dongle_id=?";
		    rc = sqlite3_prepare_v2(mDb, sqlUpdate, -1, &res, 0);
		    if (rc == SQLITE_OK) {
		        sqlite3_bind_int64(res, 1, dongle_id);
		    } else {
		        fprintf(stderr, "Failed to execute statement: %s\n", sqlite3_errmsg(mDb));
		    }

		    step = sqlite3_step(res);

		    if (step == SQLITE_DONE) {

		        printf("Dongle %8X has been found in the database \n", sqlite3_column_int(res, 0));

		    } else {
		    	puts ("This Dongle has not been found in the database");
		    }

		    sqlite3_finalize(res);

}

void SensorManager::dongleLeft(uint32_t dongle_id){

}

