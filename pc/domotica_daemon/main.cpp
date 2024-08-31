#include "Device.hpp"
#include "DeviceManager.hpp"

#include <chrono>
#include <thread>



#include "mqtt.hpp"

static DeviceManager m_dm;
DeviceManager *p_dm;
static mqqt_test *mp_mqtt;

#include "sensordata.hpp"

extern "C" {
#include "protocol.h"
#include "sensor_protocol.h"
#include "switch_protocol.h"



bscp_handler_status_t forward_handler(bscp_protocol_packet_t *data,
		protocol_transport_t transport, uint32_t param) {

	bscp_protocol_forward_t *forwarded_data =
			(bscp_protocol_forward_t*) (data->data);

//	protocol_transport_header_t flags = { .transport =
//			forwarded_data->head.transport, .from = forwarded_data->head.from,
//			.to = forwarded_data->head.to, .rssi = forwarded_data->head.rssi, };

	protocol_parse(forwarded_data->data, data->head.size
			- sizeof (protocol_transport_header_t)
			, transport,
			forwarded_data->head.as_uint32);
	return BSCP_HANDLER_STATUS_OK;
}


bscp_handler_status_t switch_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, uint32_t param) {

	puts("Switch Handler");

	protocol_transport_header_t h = { .as_uint32 = param };
	int node_id = h.from;
	uint8_t state = (packet->data[0]);
	printf("Unit %d state %d\n", node_id, state);
	mp_mqtt->publish_switch_value(node_id,state);
	// homeassistant/switch/unit_10/value
	return BSCP_HANDLER_STATUS_OK;
}

bscp_handler_status_t sensordata_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, uint32_t param) {
	bsprot_sensor_enviromental_data_t *sensordata =
			(bsprot_sensor_enviromental_data_t*) packet->data;


	// Home assistant values
	const char *device_class = nullptr;
	const char *unit_of_measurement = nullptr;

	protocol_transport_header_t h = { .as_uint32 = param };
	int unit_id = h.from;
	int sens_id = sensordata->id;
	float value_float;
	(void)value_float;
	char value[16];

	switch (sensordata->type) {
	case bsprot_sensor_enviromental_temperature:
		device_class = "temperature";
		unit_of_measurement = "°C";
		snprintf(value, sizeof(value), "%6.2f",
				(float) (sensordata->value.temperature_centi_celcius) / 100.0f);
		value_float = 		(float) (sensordata->value.temperature_centi_celcius) / 100.0f;
		break;
	case bsprot_sensor_enviromental_humidity:
		device_class = "humidity";
		unit_of_measurement = "%";
		snprintf(value, sizeof(value), "%6.1f",
				(float) (sensordata->value.humidify_relative_promille) / 10.0f);
		value_float = 		(float) (sensordata->value.humidify_relative_promille) / 10.0f;
		break;
	case bsprot_sensor_enviromental_illuminance:
		device_class = "illuminance";
		unit_of_measurement = "lux";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.illuminance_lux);
		value_float = 		(float) sensordata->value.illuminance_lux;
		break;
	case bsprot_sensor_enviromental_airpressure:
		device_class = "atmospheric_pressure";
		unit_of_measurement = "hPa";
		snprintf(value, sizeof(value), "%6.1f",
				(float) (sensordata->value.air_pressure_deci_pascal) / 10.0f);
		value_float = (float) (sensordata->value.air_pressure_deci_pascal) / 10.0f;
		break;
	case bsprot_sensor_enviromental_co2:
		device_class = "carbon_dioxide";
		unit_of_measurement = "ppm";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.co2_ppm);
		value_float = 		(float) sensordata->value.co2_ppm;
		break;
	case bsprot_sensor_enviromental_eco2:
		device_class = "carbon_dioxide"; // Best match, misses the "estimated" part
		unit_of_measurement = "ppm";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.eco2_ppm);
		value_float = 		(float) sensordata->value.eco2_ppm;
		break;
	case bsprot_sensor_enviromental_etvoc:
		device_class = "volatile_organic_compounds_parts";  // Best match, misses the "estimated" part
		unit_of_measurement = "ppb";
		snprintf(value, sizeof(value), "%7.0f",
				(float) sensordata->value.etvoc_ppb);
		value_float =(float) sensordata->value.etvoc_ppb;
		break;
	case bsprot_sensor_enviromental_pm25:
		unit_of_measurement = "µg/m³";
		device_class = "pm25";
		snprintf(value, sizeof(value), "%7.0f",	(float) sensordata->value.pm25_ugm3);
		value_float= (float) sensordata->value.pm25_ugm3;
		break;
	default:
		printf("Unknown sensor type %02X\n", sensordata->type);
		return BSCP_HANDLER_STATUS_BADDATA;
		break;
	}

	mp_mqtt->publish_sensorvalue(unit_id, sens_id, device_class, value_float);


//	mp_mqtt->publish_sensorvalue(h.from, sensordata->id, sensortype,
//			sensorvalue);
	return BSCP_HANDLER_STATUS_OK;
}
}

bscp_handler_status_t info_handler(bscp_protocol_packet_t *data,
		protocol_transport_t transport, uint32_t param) {
	puts("Received info");
	bscp_protocol_info_t * info = (bscp_protocol_info_t *)data->data;
	protocol_transport_header_t header = {.as_uint32 = param };

	int count = (data->head.size - sizeof (data->head) ) / sizeof (bscp_protocol_info_t);
	for (int i = 0 ; i < count ; i++) {
		printf("CMD %02X FLAGS %02X INDEX %02X\n",
			info[i].cmd,
			info[i].flags,
			info[i].index);

		const char * device_class = {};
		const char * unit_of_measurement = {};

		switch(info[i].cmd) {

		case BSCP_CMD_SENSOR_ENVIOREMENTAL_VALUE:
			switch(info[i].flags){
			case (1 << bsprot_sensor_enviromental_temperature):
				device_class = "temperature";
				unit_of_measurement = "°C";
					break;
			case (1 << bsprot_sensor_enviromental_illuminance):
				device_class = "illuminance";
				unit_of_measurement = "lux";
					break;
			default:
				continue;
			}
			mp_mqtt->publish_sensor(header.from, info[i].index, device_class, unit_of_measurement);
			break;
		case BSCP_CMD_SWITCH:
			mp_mqtt->publish_switch(header.from, info[i].index);
			break;
		default:
			break;
		}



	}

	return BSCP_HANDLER_STATUS_OK;
}

int main(int argc, char *argv[]) {
	protocol_register_command(info_handler, BSCP_CMD_INFO);

	protocol_register_command(forward_handler, BSCP_CMD_FORWARD);
	protocol_register_command(sensordata_handler, BSCP_CMD_SENSOR_ENVIOREMENTAL_VALUE);
	protocol_register_command(switch_handler, BSCP_CMD_SWITCH);


	mosqpp::lib_init();
	mp_mqtt = new mqqt_test();
	mp_mqtt->loop_start();

	int mid;
	int result = mp_mqtt->connect("localhost", 1883);
	printf("mqtt connect returned %d\n", result);

//	mp_mqtt->connect_async("localhost", 1883);

	m_dm.start();
	p_dm = &m_dm;
	Device * d = nullptr;

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

	std::thread(sensorDataThread, &m_dm, 0xD32A6E04, 0x10, 60).detach();

	while (1) sleep(1);

	mosqpp::lib_cleanup();
}
