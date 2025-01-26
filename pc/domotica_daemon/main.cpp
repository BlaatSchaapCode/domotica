#include "Device.hpp"
#include "DeviceManager.hpp"
#include "SensorManager.hpp"

#include <chrono>
#include <thread>

#include "mqtt.hpp"

static DeviceManager m_dm;
DeviceManager *p_dm;
mqqt_test *gp_mqtt;

#include "sensordata.hpp"

SensorManager g_sm;

extern "C" {
#include "protocol.h"
#include "sensor_protocol.h"
#include "switch_protocol.h"
typedef struct {
	protocol_transport_header_t transport;
	uint32_t dongle_id;
} forward_data_t;

bscp_handler_status_t forward_handler(bscp_protocol_packet_t *data,
		protocol_transport_t transport, void *param) {

	bscp_protocol_forward_t *forwarded_data =
			(bscp_protocol_forward_t*) (data->data);

	forward_data_t forward_data;
	forward_data.transport = *(protocol_transport_header_t*) (data->data);
	forward_data.dongle_id = *(uint32_t*) param;

	auto device = m_dm.getDevice(forward_data.dongle_id);
	switch (data->head.sub) {

	case BSCP_SUB_QSET:

		g_sm.nodeValueReset(forward_data.dongle_id,forward_data.transport.from);

		protocol_parse(forwarded_data->data,
				data->head.size - sizeof(protocol_transport_header_t),
				transport, &forward_data);
		if (device)
			device->notify_remote_response();

		g_sm.nodeValuePublish(forward_data.dongle_id,forward_data.transport.from);

		return BSCP_HANDLER_STATUS_OK;
	case BSCP_SUB_SSTA:
		printf(
				"forward handler, dongle %08X, from %02X, to %02X, status %02X\n",
				forward_data.dongle_id, forward_data.transport.from,
				forward_data.transport.to, data->head.res);
		if (device)
			device->notify_local_response(data->head.res);
		return BSCP_HANDLER_STATUS_OK;
	case BSCP_SUB_SDAT:
	case BSCP_SUB_QGET:
	default:
		return BSCP_HANDLER_STATUS_BADSUB;
	}
}

bscp_handler_status_t switch_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, void *param) {

	puts("Switch Handler");
//
	forward_data_t forward_data = *(forward_data_t*) (param);
//
	bool state = (packet->data[0]);
//	printf("Dongle %08X Unit %d State %d\n", forward_data.dongle_id,
//			forward_data.transport.from, state);
//	gp_mqtt->publish_switch_value(forward_data.transport.from, state);
//	// homeassistant/switch/unit_10/value

	g_sm.nodeValueAddSwitch(forward_data.dongle_id, forward_data.transport.from,
			0, 0, state);
	return BSCP_HANDLER_STATUS_OK;
}

bscp_handler_status_t sensordata_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, void *param) {

	forward_data_t forward_data = *(forward_data_t*) (param);

	bsprot_sensor_enviromental_data_t *sensordata =
			(bsprot_sensor_enviromental_data_t*) packet->data;

	uint32_t dongle_id = forward_data.dongle_id;
	uint8_t unit_id = forward_data.transport.from;
	uint8_t sens_id = sensordata->id;

	printf("Sensordata dongle %08X for unit %d sensor %d\n",
			forward_data.dongle_id, unit_id, sens_id);

	printf("\t\tID %02X, Type %02X, Value %4d\n", sensordata->id,
			sensordata->type, sensordata->value);

//	// Home assistant values
	const char *device_class = nullptr;
	const char *unit_of_measurement = nullptr;
	(void) unit_of_measurement;

	float value_float;
	(void) value_float;
	char value[16];

	switch (sensordata->type) {

	case bsprot_sensor_temperature:
		device_class = "temperature";
		unit_of_measurement = "°C";
		snprintf(value, sizeof(value), "%6.2f",
				(float) (sensordata->value.temperature_centi_celcius) / 100.0f);
		value_float = (float) (sensordata->value.temperature_centi_celcius)
				/ 100.0f;
		break;
	case bsprot_sensor_humidity:
		device_class = "humidity";
		unit_of_measurement = "%";
		snprintf(value, sizeof(value), "%6.1f",
				(float) (sensordata->value.humidify_relative_promille) / 10.0f);
		value_float = (float) (sensordata->value.humidify_relative_promille)
				/ 10.0f;
		break;
	case bsprot_sensor_illuminance:
		device_class = "illuminance";
		unit_of_measurement = "lux";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.illuminance_lux);
		value_float = (float) sensordata->value.illuminance_lux;
		break;
	case bsprot_sensor_airpressure:
		device_class = "atmospheric_pressure";
		unit_of_measurement = "hPa";
		snprintf(value, sizeof(value), "%6.1f",
				(float) (sensordata->value.air_pressure_deci_pascal) / 10.0f);
		value_float = (float) (sensordata->value.air_pressure_deci_pascal)
				/ 10.0f;
		break;
	case bsprot_sensor_co2:
		device_class = "carbon_dioxide";
		unit_of_measurement = "ppm";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.co2_ppm);
		value_float = (float) sensordata->value.co2_ppm;
		break;
	case bsprot_sensor_eco2:
		device_class = "carbon_dioxide"; // Best match, misses the "estimated" part
		unit_of_measurement = "ppm";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.eco2_ppm);
		value_float = (float) sensordata->value.eco2_ppm;
		break;
	case bsprot_sensor_etvoc:
		device_class = "volatile_organic_compounds_parts"; // Best match, misses the "estimated" part
		unit_of_measurement = "ppb";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.etvoc_ppb);
		value_float = (float) sensordata->value.etvoc_ppb;
		break;
	default:
		printf("Unknown sensor type %02X\n", sensordata->type);
		return BSCP_HANDLER_STATUS_BADDATA;
		break;
	}

	g_sm.nodeValueAddSensor(dongle_id, unit_id, sens_id, sensordata->type,
			value_float);

	//	mp_mqtt->publish_sensorvalue(unit_id, sens_id, device_class, value_float);
//
//	//	mp_mqtt->publish_sensorvalue(h.from, sensordata->id, sensortype,
//	//			sensorvalue);
	return BSCP_HANDLER_STATUS_OK;
}
}

bscp_handler_status_t info_handler(bscp_protocol_packet_t *data,
		protocol_transport_t transport, void *param) {
	puts("Received info");

	forward_data_t forward_data = *(forward_data_t*) (param);
	bscp_protocol_info_t *info = (bscp_protocol_info_t*) data->data;

	int count = (data->head.size - sizeof(data->head))
			/ sizeof(bscp_protocol_info_t);

	g_sm.nodeInfoReset(forward_data.dongle_id, forward_data.transport.from);

	for (int i = 0; i < count; i++) {
		switch (info[i].cmd) {
		case BSCP_CMD_SENSOR0_VALUE:
			g_sm.nodeInfoAddSensor(forward_data.dongle_id,
					forward_data.transport.from, info[i].index, info[i].flags);
			break;
		case BSCP_CMD_SWITCH:
			g_sm.nodeInfoAddSwitch(forward_data.dongle_id,
					forward_data.transport.from, info[i].index, info[i].flags);
			break;
		}
	}

	g_sm.nodeInfoPublish(forward_data.dongle_id, forward_data.transport.from);
	return BSCP_HANDLER_STATUS_OK;

}

bscp_handler_status_t info_handler_(bscp_protocol_packet_t *data,
		protocol_transport_t transport, void *param) {
	puts("Received info");
	forward_data_t forward_data = *(forward_data_t*) (param);

	bscp_protocol_info_t *info = (bscp_protocol_info_t*) data->data;

	int count = (data->head.size - sizeof(data->head))
			/ sizeof(bscp_protocol_info_t);
	for (int i = 0; i < count; i++) {
		printf("CMD %02X FLAGS %02X INDEX %02X\n", info[i].cmd, info[i].flags,
				info[i].index);

		const char *device_class = { };
		const char *unit_of_measurement = { };

		switch (info[i].cmd) {

		case BSCP_CMD_SENSOR0_VALUE:
			switch (info[i].flags) {
			case (1 << bsprot_sensor_temperature):
				device_class = "temperature";
				unit_of_measurement = "°C";
				break;
			case (1 << bsprot_sensor_illuminance):
				device_class = "illuminance";
				unit_of_measurement = "lux";
				break;
			default:
				continue;
			}
			gp_mqtt->publish_sensor(forward_data.transport.from, info[i].index,
					device_class, unit_of_measurement);
			break;
		case BSCP_CMD_SWITCH:
			gp_mqtt->publish_switch(forward_data.transport.from, info[i].index);
			break;
		default:
			break;
		}

	}

	return BSCP_HANDLER_STATUS_OK;
}

int main(int argc, char *argv[]) {

	protocol_register_command(forward_handler, BSCP_CMD_FORWARD);
	protocol_register_command(info_handler, BSCP_CMD_INFO);
	protocol_register_command(sensordata_handler,
	BSCP_CMD_SENSOR0_VALUE);
	protocol_register_command(switch_handler, BSCP_CMD_SWITCH);

	mosqpp::lib_init();
	gp_mqtt = new mqqt_test();
	gp_mqtt->loop_start();

	int mid;
	int result = gp_mqtt->connect("localhost", 1883);
	printf("mqtt connect returned %d\n", result);

	//	gp_mqtt->connect_async("localhost", 1883);

	m_dm.start();
	p_dm = &m_dm;
	Device *d = nullptr;

	g_sm.begin();

	//	bool time_synced = false;
	//	while (1) {
	//
	//		std::this_thread::sleep_for(std::chrono::seconds(5));
	//		d = m_dm.getDevice(0xD32A6E04);
	//		if (d) {
	////			puts("Getting info for 0x10");
	////			 d->testForwardGetInfo(0x10);
	//			d->setSwitch(0x10, false);
	//			puts("Off");
	//
	//		}
	//
	//		std::this_thread::sleep_for(std::chrono::seconds(5));
	//		d = m_dm.getDevice(0xD32A6E04);
	//		if (d) {
	//
	////			puts("Getting data for 0x10");
	////			 d->testForwardGetData(0x10);
	//			d->setSwitch(0x10, true);
	//			puts("On");
	//		}
	//
	//
	//	}

	//	std::thread(sensorDataThread, &m_dm, 0x03025927, 0x03, 10).detach();
	//	std::this_thread::sleep_for(std::chrono::seconds(1));
	//	std::thread(sensorDataThread, &m_dm, 0xD32A6E04, 0x10, 10).detach();

	//	std::thread(sensorDataThread, &m_dm, 0xD0226E5D, 0x06, 10).detach();

	while (1)
		sleep(1);

	mosqpp::lib_cleanup();
}
