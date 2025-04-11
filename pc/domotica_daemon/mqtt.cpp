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

#include "utils/logger.hpp"

#include "DeviceManager.hpp"
extern DeviceManager *p_dm;

mqqt_test::~mqqt_test() {}

// on_connect nto being called?
void mqqt_test::on_connect(int rc) {
  LOG_INFO(__FUNCTION__);
  int mid;
  // homeassistant/device/D0226E5D/06/command
  subscribe(&mid, "homeassistant/device/+/+/command", 2);
}

void mqqt_test::on_connect_with_flags(int rc, int flags) {
  LOG_INFO(__FUNCTION__);
}

void mqqt_test::on_disconnect(int rc) {
  LOG_INFO(__FUNCTION__);
  reconnect();
}

void mqqt_test::on_publish(int mid) { LOG_INFO(__FUNCTION__); }

void mqqt_test::on_message(const struct mosquitto_message *message) {
  LOG_INFO(__FUNCTION__);

  if (message->payloadlen < 1024) {
    // Make sure the payload is null terminated
    char payload[message->payloadlen + 1] = {};
    memcpy(payload, message->payload, message->payloadlen);
    LOG_INFO("Topic : %s", message->topic);
    LOG_INFO("Payload : %s", payload);

    // homeassistant/device/D0226E5D/06/command

    int dongle_id, node_id;

    int parsed_count = sscanf(message->topic, "homeassistant/device/%08X/%02X",
                              &dongle_id, &node_id);
    if (2 == parsed_count) {
      int state;
      sscanf(payload, "%d", &state);
      auto d = p_dm->getDevice(dongle_id);
      if (d) {
        state = !strcmp("ON", payload);
        d->setSwitch(node_id, state);
      }
    } else {
      LOG_INFO("Failed to parse topic, expect 2 values, got %d", parsed_count);
    }
  } else {
    LOG_INFO("Payload too large %d >= %d", message->payloadlen, 1024);
  }
}

void mqqt_test::on_subscribe(int mid, int qos_count, const int *granted_qos) {
  LOG_INFO(__FUNCTION__);
}

void mqqt_test::on_unsubscribe(int mid) { LOG_INFO(__FUNCTION__); }

void mqqt_test::on_log(int level, const char *str) {
  // puts(__FUNCTION__);
}

void mqqt_test::on_error() { LOG_INFO(__FUNCTION__); }

int mqqt_test::publish_sensor(int node_id, int sens_id,
                              const char *device_class,
                              const char *unit_of_measurement) {

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
  LOG_INFO("Publish configuration message mid %d status %d", mid, result);
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
    LOG_INFO("Retry: Publish value message mid %d status %d", mid, result);
  }

  return result;
}

int mqqt_test::publish_switch_value(int node_id, bool value) {
  char state_topic[256];
  snprintf(state_topic, sizeof(state_topic),
           "homeassistant/switch/unit_%02x/value", node_id);

  int mid;
  char val[] = {(char)('0' + value)};
  int result = publish(&mid, state_topic, 1, val);
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
  LOG_INFO("Publish configuration message mid %d status %d", mid, result);
  return result;
}
