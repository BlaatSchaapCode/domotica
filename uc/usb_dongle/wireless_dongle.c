/*

 File: 		wireless_sensor.c
 Author:	André van Schoubroeck
 License:	MIT


 MIT License

 Copyright (c) 2023 André van Schoubroeck <andre@blaatschaap.be>

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.

 */

#include <stdbool.h>
#include <string.h>

#include "system.h"

// NB. On STM32F0, stdfix conflicts with
// STM32CubeF0/Drivers/CMSIS/Core/Include/cmsis_gcc.h
// It should be included after STM32 includes stm32f0xx.h (included by system.h)
#include <stdfix.h>
// Might need to verify this also holds for latest CMSIS, and switch to upstream

#include <stdio.h>

#include "SEGGER_RTT.h"
#include "bshal_delay.h"
#include "bshal_gpio_stm32.h"
#include "bshal_i2cm.h"
#include "bshal_i2cm_stm32.h"
#include "bshal_spim.h"
#include "protocol.h"
#include "sensor_protocol.h"
//#include "spi_flash.h"
#include "i2c_eeprom.h"
#include "sxv1.h"
#include "timer.h"
#include "usbd.h"

static bshal_i2cm_instance_t m_i2c;
bshal_i2cm_instance_t *gp_i2c = &m_i2c;
bshal_spim_instance_t spi_flash_config;
bsradio_instance_t m_radio;
bscp_usbd_handle_t *gp_usbd;
i2c_eeprom_t i2c_eeprom_config;

bshal_i2cm_instance_t *i2c_init(void) {
#ifdef STM32
	m_i2c.sda_pin = bshal_gpio_encode_pin(GPIOB, GPIO_PIN_7);
	m_i2c.scl_pin = bshal_gpio_encode_pin(GPIOB, GPIO_PIN_6);
	m_i2c.hw_nr = 1;
#elif defined GECKO
	//	m_i2c.sda_pin = 5;// bshal_gpio_encode_pin(gpioPortA, 5);
	//	m_i2c.scl_pin = 4;// bshal_gpio_encode_pin(gpioPortA, 4);

	m_i2c.sda_pin = bshal_gpio_encode_pin(1, 2);
	m_i2c.scl_pin = bshal_gpio_encode_pin(1, 1);
	m_i2c.hw_nr = 0;
#endif

	m_i2c.speed_hz = 100000;
	//	m_i2c.speed_hz = 400000;
	// m_i2c.speed_hz = 360000;
#ifdef STM32
	bshal_stm32_i2cm_init(&m_i2c);
#elif defined GECKO
	bshal_gecko_i2cm_init(&m_i2c);
#else
#error "Unsupported MCU"
#endif
	return &m_i2c;
}


void i2c_eeprom_init(void) {
	// Configuration of 2 Kbit (256 bytes)
	// xx24C02
	i2c_eeprom_config.p_i2c = gp_i2c;
	i2c_eeprom_config.i2c_addr = 0x50;
	i2c_eeprom_config.page_count = 32;
	i2c_eeprom_config.page_size = 8;
	i2c_eeprom_config.page_address_size = 1;
}



int radio_init(bsradio_instance_t *bsradio) {
	i2c_eeprom_init();

	bsradio->spim.frequency = 1000000;
	bsradio->spim.bit_order = 0; // MSB
	bsradio->spim.mode = 0;

	bsradio->spim.hw_nr = 1;
	bsradio->spim.sck_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_5);
	bsradio->spim.miso_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_6);
	bsradio->spim.mosi_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_7);
	bsradio->spim.cs_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_4);
	bsradio->spim.rs_pin = bshal_gpio_encode_pin(GPIOB, GPIO_PIN_10);

	bshal_spim_init(&bsradio->spim);

	uint8_t hwconfig_buffer[0x14] = { };
	bscp_protocol_header_t *header = (bscp_protocol_header_t*) (hwconfig_buffer);
	bsradio_hwconfig_t *hwconfig = (bsradio_hwconfig_t*) (hwconfig_buffer
			+ sizeof(bscp_protocol_header_t));

	int result = i2c_eeprom_read(&i2c_eeprom_config, 0x00, hwconfig_buffer,
			sizeof(hwconfig_buffer));
	(void)result;
	if (header->size
			== sizeof(bscp_protocol_header_t) + sizeof(bsradio_hwconfig_t)) {
		bsradio->hwconfig = *hwconfig;
		puts("hwconfig loaded");
	} else {
		puts("hwconfig missing");
		//		memset(buffert, 0xFF, sizeof(buffert));
		header->size = sizeof(bscp_protocol_header_t)
				+ sizeof(bsradio_hwconfig_t);
		header->cmd = 0x02;
		header->sub = 0x20;
		header->res = 'R';

		hwconfig->chip_brand = chip_brand_semtech;
		hwconfig->chip_type = 1;
		hwconfig->chip_variant = -1;
		hwconfig->module_brand = module_brand_dreamlnk;
		hwconfig->module_variant = -1;
		hwconfig->frequency_band = 868;
		hwconfig->tune = 0;
		hwconfig->pa_config = 1;
		hwconfig->antenna_type = -1;
		hwconfig->xtal_freq = 32000000;

		hwconfig->tune = -1;

		bsradio->hwconfig = *hwconfig;

		bool update_flash = false;

		if (update_flash) {
			i2c_eeprom_program(&i2c_eeprom_config, 0x000, hwconfig_buffer,
					sizeof(hwconfig_buffer));
			puts("Done");
		}
	}

	// Need to write this first
	uint8_t rfconfig_buffer[0x23] = { };
	header = (bscp_protocol_header_t*) (rfconfig_buffer);
	bsradio_rfconfig_t *rfconfig = (bsradio_rfconfig_t*) (rfconfig_buffer
			+ sizeof(bscp_protocol_header_t));
	i2c_eeprom_read(&i2c_eeprom_config, 0x14, rfconfig_buffer, 0x23);


	if (header->size
					== sizeof(bscp_protocol_header_t)
							+ sizeof(bsradio_rfconfig_t)) {
		bsradio->rfconfig = *rfconfig;
		puts("rfconfig loaded");
	} else {
		puts("rfconfig missing");

		switch (bsradio->hwconfig.frequency_band) {
		case 434:
			bsradio->rfconfig.frequency_kHz = 434000;
			bsradio->rfconfig.tx_power_dBm = 10;
			break;
		case 868:
			bsradio->rfconfig.frequency_kHz = 869850;
			bsradio->rfconfig.tx_power_dBm = 7;
			break;
		case 915:
			bsradio->rfconfig.frequency_kHz = 915000;
			bsradio->rfconfig.tx_power_dBm = -3;
			break;
		}

		bsradio->rfconfig.modulation_shaping = 5; // 0.5 gfsk
		bsradio->rfconfig.modulation = modulation_2fsk;

		//		bsradio->rfconfig.birrate_bps = 12500;
		//		bsradio->rfconfig.freq_dev_hz = 12500;
		//		bsradio->rfconfig.bandwidth_hz = 25000;

		//		We want to do higher speed in the future, but for now
		//		Let's get the I²C EEPROM for the settings working first
		bsradio->rfconfig.birrate_bps = 25000;
		bsradio->rfconfig.freq_dev_hz = 25000;
		bsradio->rfconfig.bandwidth_hz = 50000;

		//		bsradio->rfconfig.birrate_bps = 50000;
		//		bsradio->rfconfig.freq_dev_hz = 50000;
		//		bsradio->rfconfig.bandwidth_hz = 100000;

		extern uint32_t get_serial(void);
		(*(uint32_t *)bsradio->rfconfig.network_id) = get_serial();
		bsradio->rfconfig.network_id_size = 4;
		bsradio->rfconfig.node_id = 0;

		bsradio->rfconfig.node_id = 0x00;
		bsradio->rfconfig.broadcast_id = 0xFF;

		bool update_flash = false;

		if (update_flash) {
			header->size = sizeof(bscp_protocol_header_t)
					+ sizeof(bsradio_rfconfig_t);
			header->cmd = 0x02;
			header->sub = 0x20;
			header->res = 'r';
			*rfconfig = (bsradio->rfconfig);

			i2c_eeprom_program(&i2c_eeprom_config, 0x14, rfconfig_buffer, 0x23);
			puts("Done");
		}
	}

	if (chip_brand_semtech == bsradio->hwconfig.chip_brand
			&& bsradio->hwconfig.chip_type == 1) {
		bshal_gpio_write_pin(bsradio->spim.rs_pin, 1);
		bshal_delay_ms(5);
		bshal_gpio_write_pin(bsradio->spim.rs_pin, 0);
		bshal_delay_ms(50);

		bsradio->driver.set_frequency = sxv1_set_frequency;
		bsradio->driver.set_tx_power = sxv1_set_tx_power;
		bsradio->driver.set_bitrate = sxv1_set_bitrate;
		bsradio->driver.set_fdev = sxv1_set_fdev;
		bsradio->driver.set_bandwidth = sxv1_set_bandwidth;
		bsradio->driver.init = sxv1_init;
		bsradio->driver.set_network_id = sxv1_set_network_id;
		bsradio->driver.set_mode = sxv1_set_mode;
		bsradio->driver.recv_packet = sxv1_recv_packet;
		bsradio->driver.send_packet = sxv1_send_packet;
	} else
		return -1;



	  puts("Radio config");
	  printf("Bandwidth:      %6d Hz\n", bsradio->rfconfig.bandwidth_hz);
	  printf("Bitrate:        %6d bps\n", bsradio->rfconfig.birrate_bps);
	  printf("Freq. dev.      %6d Hz \n", bsradio->rfconfig.freq_dev_hz);
	  printf("Frequency       %6d kHz\n", bsradio->rfconfig.frequency_kHz);
	  printf("Network id:     %08lX\n", *(uint32_t*)(bsradio->rfconfig.network_id));
	  printf("Node id:        %d\n", bsradio->rfconfig.node_id);

	bsradio_init(bsradio);

	char chip_version;
	sxv1_read_reg(bsradio, SXV1_REG_VERSION, &chip_version);

	switch (chip_version) {
	case 0x23:
		puts("SX123x : OK");
		break;
	case 0x24:
		puts("SX1231H: OK");
		break;
	default:
		puts("SX123x : FAIL");
		break;
	}

}


bscp_handler_status_t forward_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport,
		void* param) {
	bscp_protocol_forward_t *forward = (bscp_protocol_forward_t *)(packet->data);

	if (packet->head.sub == BSCP_SUB_QSET) {
		switch (forward->head.transport) {
		case PROTOCOL_TRANSPORT_RF:
			bsradio_packet_t request = {}, response = {};
			request.from = 0x00;
			request.to = forward->head.to;
			request.length = packet->head.size - sizeof(packet->head);
#pragma pack(push, 1)
			memcpy(request.payload, forward->data,
					packet->head.size - sizeof(packet->head));
			int result = bsradio_send_request(&m_radio, &request, &response);
			if (transport == PROTOCOL_TRANSPORT_USB) {
				static bscp_protocol_packet_t usb_response;
				bscp_protocol_forward_t* forward_response = (bscp_protocol_forward_t*)(usb_response.data);
				forward_response->head = forward->head;
				usb_response.head = packet->head;
				usb_response.head.size = sizeof (forward->head) + sizeof (usb_response.head);
				usb_response.head.res = result;
				usb_response.head.sub = BSCP_SUB_SSTA;
				bscp_usbd_transmit(gp_usbd, 0x81, &usb_response,
						usb_response.head.size);
			}

#pragma pack(pop)

return BSCP_HANDLER_STATUS_OK_FW;
		default:
			return BSCP_HANDLER_STATUS_BADDATA;
		}
	}
	return BSCP_HANDLER_STATUS_ERROR;
}

bscp_handler_status_t sensordata_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport,
		void* param) {
	bsprot_sensor_enviromental_data_t *sensordata =
			(bsprot_sensor_enviromental_data_t *)packet->data;
	printf("Sensor %2d ", sensordata->id);
	switch (sensordata->type) {
	case bsprot_sensor_enviromental_temperature:
		printf("%16s %3d.%2d °C", "temperature",
				sensordata->value.temperature_centi_celcius / 100,
				abs(sensordata->value.temperature_centi_celcius) % 100);
		break;
	case bsprot_sensor_enviromental_humidity:
		printf("%16s ", "humidity");

		break;
	case bsprot_sensor_enviromental_illuminance:
		printf("%16s ", "illuminance");

		break;
	case bsprot_sensor_enviromental_airpressure:
		printf("%16s ", "airpressure");
		break;

	case bsprot_sensor_enviromental_co2:
		printf("%16s ", "co2");

		break;
	case bsprot_sensor_enviromental_eco2:
		printf("%16s ", "eco2");

		break;
	case bsprot_sensor_enviromental_etvoc:
		printf("%16s ", "etvoc");

		break;
	case bsprot_sensor_enviromental_pm25:

		break;
	default:
		break;
	}
	putchar('\n');

	return BSCP_HANDLER_STATUS_OK;
}

int main() {
	extern char* get_serial_string();
	SEGGER_RTT_Init();
	puts("Wireless Dongle");
	printf("Serial: %s\n", get_serial_string());

	// New GCC versions are nit-picking.
	// Put them here for now, make it neat later.
	extern void ClockSetup_HSE8_SYS72(void);
	extern void ClockSetup_HSE8_SYS48(void);
	extern void usbd_reenumerate(void);
	extern void usbd_process(void);

	ClockSetup_HSE8_SYS72();
	SystemCoreClockUpdate();

	usbd_reenumerate();
	gp_usbd = usbd_init();
	timer_init();
	i2c_init();

	radio_init(&m_radio);
	bsradio_set_mode(&m_radio, mode_receive);

	protocol_register_command(sensordata_handler,
			BSCP_CMD_SENSOR0_VALUE);

	protocol_register_command(forward_handler, BSCP_CMD_FORWARD);
	while (1) {


		bool calibration = false;
		if (calibration) {
			bsradio_packet_t request = { }, response = { };
			m_radio.hwconfig.tune = 0;
			while (1) {

				sxv1_set_mode_internal(&m_radio, sxv1_mode_standby);

				// Calibration to fund the tune value, for SXv1 (RFM69)
				// this is the frequency offset in kHz
				(void)m_radio.hwconfig.tune;

				sxv1_set_frequency(&m_radio, 870000);

				response.ack_request = 0;
				response.ack_response = 1;
				response.to = request.from;
				response.from = request.to;
				response.length = 4;

				puts("Sending ACK");
				bsradio_send_packet(&m_radio, &response);
				bshal_delay_ms(1000);
			}
		}


		usbd_process();

		bsradio_packet_t request = {}, response = {};
		memset(&request, 0, sizeof(request));

		if (!bsradio_recv_packet(&m_radio, &request)) {
			puts("Packet received");
			printf("Length %2d, to: %02X, from: %02X rssi %3d\n", request.length,
					request.to, request.from, request.rssi);
			if (request.ack_request) {
				response = request;
				response.length = 4;
				response.ack_request = 0;
				response.ack_response = 1;
				response.to = request.from;
				response.from = request.to;
				bsradio_send_packet(&m_radio, &response);
			}

			// protocol_parse(request.payload, request.length,PROTOCOL_TRANSPORT_RF,
			// request.rssi);

			// Forward
			static char buffer[255] = {};
			bscp_protocol_packet_t *forward_packet =
					(bscp_protocol_packet_t *)(buffer);
			bscp_protocol_forward_t *forward_data =
					(bscp_protocol_forward_t *)(forward_packet->data);
			*((bsradio_packet_t *)forward_data->data) = request;

			forward_packet->head.size = sizeof(bscp_protocol_packet_t) +
					sizeof(bscp_protocol_forward_t) +
					sizeof(request.payload);
			forward_packet->head.cmd = BSCP_CMD_FORWARD;
			//forward_packet->head.sub = BSCP_SUB_SDAT;
			forward_packet->head.sub = BSCP_SUB_QSET;

			forward_data->head.transport = PROTOCOL_TRANSPORT_RF;
			forward_data->head.from = request.from;
			forward_data->head.to = request.to;
			forward_data->head.rssi = request.rssi;

			//			if (request.from != 1)
				//				__BKPT(0);

			// memcpy ( forward_data->data, request.payload, request.length);
			memcpy(forward_data->data, request.payload, sizeof(request.payload));

			bscp_usbd_transmit(gp_usbd, 0x81, forward_packet,
					forward_packet->head.size);

			memset(&request, 0, sizeof(request));
			memset(&response, 0, sizeof(response));
		}
	}
}

uint32_t get_network_id(void) {
	return *((uint32_t*) m_radio.rfconfig.network_id);
}

uint8_t get_node_id(void) {
	return m_radio.rfconfig.node_id;
}

void enter_normal_mode(void) {
	bsradio_init(&m_radio);
}

void enter_pair_mode(void) {
	static bsradio_instance_t radio_pairmode;
	radio_pairmode = m_radio;
	(*(uint32_t*) radio_pairmode.rfconfig.network_id) = 0xdeadbeef;
	radio_pairmode.rfconfig.node_id = 0x00;
	bsradio_init(&radio_pairmode);
}
