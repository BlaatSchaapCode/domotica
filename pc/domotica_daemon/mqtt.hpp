/*
 * mqqt.hpp
 *
 *  Created on: 24 dec. 2023
 *      Author: andre
 */

#ifndef MQTT_HPP_
#define MQTT_HPP_

#include <string>

#include <mosquittopp.h>

class mqqt_test: public mosqpp::mosquittopp {
public:
//		int connect_to_server(const char *id, const char *host, int port);
	~mqqt_test();

	 void on_connect(int rc) override;
	 void on_connect_with_flags(int rc, int flags) override;
	 void on_disconnect(int rc) override;
	 void on_publish(int mid) override;
	 void on_message(const struct mosquitto_message * message) override;
	 void on_subscribe(int mid, int qos_count, const int * granted_qos) override;
	 void on_unsubscribe(int mid) override;
	 void on_log(int level, const char * str) override;
	 void on_error() override;



	int publish_sensor(int node_id, int sens_id, const char *device_class,
				const char *unit_of_measurement);

	int publish_sensorvalue(int node_id, int sens_id, const char *device_class, float value);

	int publish_switch(int node_id, int switch_id);

	int publish_switch_value(int node_id, bool value) ;

private:
	std::string m_id;
	std::string m_host;
	int m_port;



};

#endif /* MQTT_HPP_ */
