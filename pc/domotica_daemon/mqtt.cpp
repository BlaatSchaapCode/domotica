/*
 * mqqt.cpp
 *
 *  Created on: 24 dec. 2023
 *      Author: andre
 */

#include <cstdio>
#include <cstring>
#include <thread>

#include "mqtt.hpp"
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

#include "DeviceManager.hpp"
extern DeviceManager *p_dm;

//int mqqt_test::connect_to_server(const char *id, const char *host, int port) {
//
//	int keepalive = 60;
//	mosquittopp(id, true);
//	return connect(host, port, keepalive);
//}

mqqt_test::~mqqt_test() {
}


// on_connect nto being called?
void mqqt_test::on_connect(int rc) {
	puts(__FUNCTION__);
	int mid;
	subscribe(&mid, "homeassistant/switch/+/set/#", 2);
}

void mqqt_test::on_connect_with_flags(int rc, int flags) {
	puts(__FUNCTION__);
}

void mqqt_test::on_disconnect(int rc) {
	puts(__FUNCTION__);
	reconnect();
}

void mqqt_test::on_publish(int mid) {
	puts(__FUNCTION__);
}

void mqqt_test::on_message(const struct mosquitto_message *message) {
	puts(__FUNCTION__);

	if (message->payloadlen < 1024) {
		// Make sure the payload is null terminated
		char payload [message->payloadlen + 1] = {};
		memcpy(payload, message->payload, message->payloadlen);
		printf("Topic : %s \n" , message->topic);
		printf("Payload : %s \n" , payload);

		int node_id, switch_id;
		if (2 == sscanf(message->topic, "homeassistant/switch/unit_%02X/set/%02X", &node_id, &switch_id)) {
			int state;
			sscanf(payload, "%d", &state);
			// TODO, SensorManager to derive dongle id
			auto d = p_dm->getDevice(0xD32A6E04);
			if (d) {
				d->setSwitch(node_id, state);
			}
		}
	}
}

void mqqt_test::on_subscribe(int mid, int qos_count, const int *granted_qos) {
	puts(__FUNCTION__);
}

void mqqt_test::on_unsubscribe(int mid) {
	puts(__FUNCTION__);
}

void mqqt_test::on_log(int level, const char *str) {
	//puts(__FUNCTION__);
}

void mqqt_test::on_error() {
	puts(__FUNCTION__);
}

int mqqt_test::publish_sensor(int node_id, int sens_id,
		const char *device_class, const char *unit_of_measurement) {

	char config_topic[256];
	char state_topic[256];
	nlohmann::json json_config;
	nlohmann::json json_value;

	snprintf(config_topic, sizeof(config_topic),
			"homeassistant/sensor/unit_%02x/%s_%02x/config", node_id,
			device_class, sens_id);
	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/sensor/unit_%02x/value", node_id);

	char value_template[256];
	snprintf(value_template, sizeof(value_template), "{{value_json.%s_%02x}}",
			device_class, sens_id);

	json_config["device_class"] = device_class;
	json_config["state_topic"] = state_topic;
	json_config["unit_of_measurement"] = unit_of_measurement;
	json_config["value_template"] = value_template;

	char unique_id[32];
	snprintf(unique_id, sizeof(unique_id), "sensor/%02X/%02X", node_id & 0xFF,
			sens_id & 0xFF);
	json_config["unique_id"] = unique_id;

	char name_buff[32];
	snprintf(name_buff, sizeof(name_buff), "%02X", node_id & 0xFF);
	json_config["device"]["identifiers"] = name_buff;

	int mid;
	auto json_config_dump = json_config.dump();
	int result = publish(&mid, config_topic, json_config_dump.length(),
			json_config_dump.c_str());
	printf("Publish configuration message mid %d status %d\n", mid, result);
	return result;

}

int mqqt_test::publish_sensorvalue(int node_id, int sens_id,
		const char *device_class, float value) {
	char state_topic[256];
	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/sensor/unit_%02x/value", node_id);
	nlohmann::json json_value;
	char value_name[256];
	snprintf(value_name, sizeof(value_name), "%s_%02x", device_class, sens_id);
	json_value[value_name] = value;
	auto json_value_dump = json_value.dump();
	int mid;
	int result = publish(&mid, state_topic, json_value_dump.length(),
			json_value_dump.c_str());
	printf("Publish value message mid %d status %d\n", mid, result);
	if (14 == result) {
		reconnect();
		result = publish(&mid, state_topic, json_value_dump.length(),
					json_value_dump.c_str());
		printf("Retry: Publish value message mid %d status %d\n", mid, result);
	}

	return result;
}

int mqqt_test::publish_switch_value(int node_id, bool value) {
	char state_topic[256];
	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/switch/unit_%02x/value", node_id);

	int mid;
	char val[] = {'0' + value};
	int result = publish(&mid, state_topic, 1,
			val);
	return result;

}

int mqqt_test::publish_switch(int node_id, int switch_id) {

	char config_topic[256];
	char state_topic[256];
	char command_topic[256];
	nlohmann::json json_config;
	nlohmann::json json_value;

	snprintf(config_topic, sizeof(config_topic),
			"homeassistant/switch/unit_%02x/switch_%02x/config", node_id,
			switch_id);
	snprintf(state_topic, sizeof(state_topic),
			"homeassistant/switch/unit_%02x/value", node_id);

	snprintf(command_topic, sizeof(command_topic),
			"homeassistant/switch/unit_%02x/set/%02x", node_id, switch_id);

	json_config["state_topic"] = state_topic;
	json_config["command_topic"] = command_topic;

	json_config["payload_on"] = 1;
	json_config["payload_off"] = 0;
	json_config["state_on"] = 1;
	json_config["state_off"] = 0;

//	char value_template[256];
//	snprintf(value_template, sizeof(value_template),
//			"{{value_json.switch_%02x}}", switch_id);

	char unique_id[32];
	snprintf(unique_id, sizeof(unique_id), "switch/%02X/%02X", node_id & 0xFF,
			switch_id & 0xFF);
	json_config["unique_id"] = unique_id;

	char name_buff[32];
	snprintf(name_buff, sizeof(name_buff), "%02X", node_id & 0xFF);
	json_config["device"]["identifiers"] = name_buff;

	int mid;
	auto json_config_dump = json_config.dump();
	int result = publish(&mid, config_topic, json_config_dump.length(),
			json_config_dump.c_str());
	printf("Publish configuration message mid %d status %d\n", mid, result);
	return result;

}
