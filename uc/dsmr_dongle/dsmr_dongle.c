/*

 File: 		wireless_sensor.c
 Author:	André van Schoubroeck
 License:	MIT


 MIT License

 Copyright (c) 2023 - 2024 André van Schoubroeck <andre@blaatschaap.be>

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
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "system.h"

// NB. On STM32F0, stdfix conflicts with
// STM32CubeF0/Drivers/CMSIS/Core/Include/cmsis_gcc.h
// It should be included after STM32 includes stm32f0xx.h (included by system.h)
#include <stdfix.h>
// Might need to verify this also holds for latest CMSIS, and switch to upstream

#include "bshal_delay.h"
#include "bshal_gpio.h"
#include "bshal_i2cm.h"
#include "bshal_spim.h"

#include "lm75b.h"
#include "sxv1.h"

#include "protocol.h"
#include "sensor_protocol.h"
#include "switch_protocol.h"
#include "time_protocol.h"

#include <bshal_gpio_stm32.h>
#include <bshal_i2cm.h>
#include <bshal_i2cm_stm32.h>

#include "hal.h"
// #include "spi_flash.h"
#include "i2c_eeprom.h"

#include "display.h"
#include "ir.h"
#include "rtc.h"
#include "dsmr.h"
#include "timer.h"
#include "serial.h"
#include "sxv1.h"

#include "pair_protocol.h"

#include <SEGGER_RTT.h>

static bshal_i2cm_instance_t m_i2c;
bshal_i2cm_instance_t *gp_i2c = NULL;
// bshal_spim_instance_t spi_flash_config;
i2c_eeprom_t i2c_eeprom_config;
static bsradio_instance_t m_radio;
bsradio_instance_t *gp_radio = NULL;

bshal_i2cm_instance_t* i2c_init(void) {
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
	gp_i2c = &m_i2c;
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

int radio_write_rfconfig(void) {
	uint8_t rfconfig_buffer[0x23] = { };
	bscp_protocol_header_t *header = (bscp_protocol_header_t*) (rfconfig_buffer);
	bsradio_rfconfig_t *rfconfig = (bsradio_rfconfig_t*) (rfconfig_buffer
			+ sizeof(bscp_protocol_header_t));

	header->size = sizeof(bscp_protocol_header_t) + sizeof(bsradio_rfconfig_t);
	header->cmd = 0x02;
	header->sub = 0x20;
	header->res = 'r';
	*rfconfig = m_radio.rfconfig;

	i2c_eeprom_program(&i2c_eeprom_config, 0x14, rfconfig_buffer,
			sizeof(rfconfig_buffer));
	puts("Done");
	return 0;

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
	(void) result;
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
		hwconfig->antenna_type = antenna_type_trace;
		hwconfig->xtal_freq = 32000000;

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

	if (header->size  // disabled for debug
	== sizeof(bscp_protocol_header_t) + sizeof(bsradio_rfconfig_t)) {
		bsradio->rfconfig = *rfconfig;
		puts("rfconfig loaded");

		// Update settings in existing unit
		bsradio->rfconfig.crc = CCITT_16;
		bsradio->rfconfig.birrate_bps = 50000;
		bsradio->rfconfig.freq_dev_hz = 50000;
		bsradio->rfconfig.bandwidth_hz = 100000;

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
		bsradio->rfconfig.crc = CCITT_16;

		bsradio->rfconfig.birrate_bps = 50000;
		bsradio->rfconfig.freq_dev_hz = 50000;
		bsradio->rfconfig.bandwidth_hz = 100000;

		bsradio->rfconfig.network_id_size = 4;

		bsradio->rfconfig.node_id = 0x25;
		bsradio->rfconfig.broadcast_id = 0xFF;

		bool update_flash = false;
		//__BKPT(0);
		if (update_flash) {
			radio_write_rfconfig();
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
	printf("Bandwidth:      %6ld Hz\n", bsradio->rfconfig.bandwidth_hz);
	printf("Bitrate:        %6ld bps\n", bsradio->rfconfig.birrate_bps);
	printf("Freq. dev.      %6ld Hz \n", bsradio->rfconfig.freq_dev_hz);
	printf("Frequency       %6ld kHz\n", bsradio->rfconfig.frequency_kHz);
	printf("Tune offset     %6ld kHz\n", bsradio->hwconfig.tune);
	printf("Network id:     %08lX\n",
			*(uint32_t*) (bsradio->rfconfig.network_id));
	printf("Node id:        %d\n", bsradio->rfconfig.node_id);

	bsradio_init(bsradio);

	char chip_version;
	sxv1_read_reg(bsradio, SXV1_REG_VERSION, &chip_version);

	switch (chip_version) {
	case 0x23:
		puts("SX123x : OK");
		return 0;
	case 0x24:
		puts("SX1231H: OK");
		return 0;
	default:
		puts("SX123x : FAIL");
		return -1;
	}

}

void SysTick_Handler(void) {
	// Make the STM32 HAL happy.
	HAL_IncTick();
}

void deviceinfo_send() {
	// todo
}

bscp_handler_status_t info_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, void *param) {

	if (packet->head.sub == BSCP_SUB_QGET)
		deviceinfo_send();
	return 0;
}

void gpio_init() {
	// int bshal_gpio_cfg_in(uint8_t bshal_pin, gpio_drive_type_t drive_type, bool
	// init_val){
	bshal_gpio_cfg_in(0, opendrain, 1);
	bshal_gpio_cfg_in(2, opendrain, 1);
	bshal_gpio_cfg_out(1, pushpull, 0);
}

bscp_handler_status_t pair_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, void *param) {
	switch (packet->head.sub) {
	case BSCP_SUB_QGET:
		return BSCP_HANDLER_STATUS_BADSUB;
	case BSCP_SUB_QSET:

	{
		pairing_t *pairing = (pairing_t*) (packet->data);
		m_radio.rfconfig.network_id_size = 4;
		(*(uint32_t*) (m_radio.rfconfig.network_id)) = pairing->network_id;
		m_radio.rfconfig.node_id = pairing->node_id;
		m_radio.rfconfig.broadcast_id = 0xFF;

		puts("Done");

	}
		//--

		return BSCP_HANDLER_STATUS_OK;
	default:
		return BSCP_HANDLER_STATUS_BADSUB;
	}
}

bscp_handler_status_t unixtime_handler(bscp_protocol_packet_t *packet,
		protocol_transport_t transport, void *param) {

	time_t unixtime = *(uint32_t*) packet->data;
	switch (packet->head.sub) {
	case BSCP_SUB_QGET:
		// TODO
		return 0;
	case BSCP_SUB_QSET:
		time_set(unixtime);
		return 0;
		break;
	default:
		return 0;
	}
}

void calibrate_clock() {
	display_clear();
	display_print_upper("Calibrating");
	display_print_middle("  Clock");
	display_apply();

	time_t t = time(NULL);
	while (t == time(NULL))
		;
	t = time(NULL);
	uint32_t a = get_time_ms();
	while (t == time(NULL))
		;
	uint32_t b = get_time_ms();
	uint32_t c = b - a;
	printf("clock speed deviation: %ld\n", c);
}

void radio_calibrate(void) {
	bsradio_packet_t response = { };
	bool calibration = false;
	if (calibration) {
		int freq = 870000;
		m_radio.hwconfig.tune = 0;
		while (1) {

			sxv1_set_mode_internal(&m_radio, sxv1_mode_standby);

			// Calibration to fund the tune value, for SXv1 (RFM69)
			// this is the frequency offset in kHz
			(void) m_radio.hwconfig.tune;

			sxv1_set_frequency(&m_radio, freq);

			response.ack_request = 0;
			response.ack_response = 1;
			response.to = 0xFF;
			response.from = 0xFF;
			response.length = 4;

			puts("Sending Packet");
			bsradio_send_packet(&m_radio, &response);
			bshal_delay_ms(1000);
		}
	}
}
void radio_process(void) {
	bsradio_packet_t request = { }, response = { };

	if (!bsradio_recv_packet(&m_radio, &request)) {

		if (request.ack_request) {
			printf(
					"request:   seq %3d try %3d len %2d, from: %02X, to  : %02X\n",
					request.seq_nr, request.retry_cnt, request.length,
					request.from, request.to);
		} else if (request.ack_response) {
			printf(
					"response:  seq %3d try %3d len %2d, to  : %02X, from: %02X\n",
					request.seq_nr, request.retry_cnt, request.length,
					request.to, request.from);
		} else {
			puts("????");
		}

		// This filter will be moved to bsradio later
		// possible to hardware level if supported by radio.
		if (request.to == m_radio.rfconfig.node_id
		// 0xFE is pair mode. Quick and Dirty handling
				|| request.to == 0xFE) {
			puts("Packet is for us");

			if (request.ack_request) {
				response = request;
				response.ack_request = 0;
				response.ack_response = 1;
				response.to = request.from;
				response.from = request.to;
				response.length = 4;

				puts("Sending ACK");
				bsradio_send_packet(&m_radio, &response);

				puts("Processing packet");
				protocol_transport_header_t flags = { .from = request.from,
						.to = request.to, .rssi = request.rssi, .transport =
								PROTOCOL_TRANSPORT_RF };

				bscp_protocol_packet_t *payload =
						(bscp_protocol_packet_t*) (request.payload);
				printf("\tSize %2d, cmd: %02X, sub: %02X, res: %02X\n",
						payload->head.size, payload->head.cmd,
						payload->head.sub, payload->head.res);

				protocol_parse(request.payload, request.length,
						PROTOCOL_TRANSPORT_RF, &flags);
			}
		} else {
			puts("Packet is not for us");
		}
		memset(&request, 0, sizeof(request));
		memset(&response, 0, sizeof(response));
	}
}

void timekeeper(char *func) {
	static uint32_t ts = 0;
	if (ts) {
		uint32_t time_taken = get_time_ms() - ts;
		if (time_taken > 5)
			printf("%16s took %6d ms\n", func, time_taken);
	}
	ts = get_time_ms();
}

void radio_irq_handler(void) {
	puts("IRQ!!!");
//	printf("IRQ PIN %d\n",bshal_gpio_read_pin(27));
//	radio_process();
}
void radio_irq_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	__HAL_RCC_GPIOB_CLK_ENABLE();

	//GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

char recvbuffer[2048] = { }; // do we have enough ram
char procbuffer[2048] = { }; // do we have enough ram
char donebuffer[2048] = { }; // do we have enough ram
unsigned recvpos = 0;
unsigned procpos = 0;
bool synced = false;
bool ready = false;

meterstand_t meterstand = { };

void dsmr_process() {
	if (ready) {
		meterstand = dsmr_parse(donebuffer, strlen(donebuffer));
		memset(donebuffer, 0, sizeof(donebuffer));
		ready = false;
	}

}
static UART_HandleTypeDef m_uart;
void uart_init() {
	// For now, directly against STM HAL?
	UART_HandleTypeDef *handle = &m_uart;
	handle->Instance = USART2;
	__HAL_RCC_USART2_CLK_ENABLE();
	NVIC_EnableIRQ(USART2_IRQn);

	handle->Init.BaudRate = 115200;
	handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	handle->Init.Mode = UART_MODE_RX;
	handle->Init.OverSampling = UART_OVERSAMPLING_16;
	handle->Init.Parity = UART_PARITY_NONE;
	handle->Init.WordLength = UART_WORDLENGTH_8B;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Data Request Line (Connected to CTS)
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, true);

	HAL_UART_Init(handle);

	HAL_UARTEx_ReceiveToIdle_IT(handle, recvbuffer, sizeof(recvbuffer));
}

void USART2_IRQHandler(void) {
	HAL_UART_IRQHandler(&m_uart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

	UART_HandleTypeDef *handle = &m_uart;

	char *begin = strstr(recvbuffer, "/");
	char *end = strstr(recvbuffer, "!");
	if (begin) {
		procpos = 0;
		int inpos = begin - recvbuffer;
		int remaining = ((int) Size - inpos);
		if (remaining > 0) {
			memset(procbuffer, 0, sizeof(procbuffer));
			memcpy(procbuffer, recvbuffer + inpos, remaining);
			procpos = remaining;
		}
	} else {
		if ((procpos + Size) < sizeof(procbuffer)) {
			memcpy(procbuffer + procpos, recvbuffer, Size);
			procpos += Size;
		}
		if (end) {
			memcpy(donebuffer, procbuffer, sizeof(donebuffer));
			ready = true;
		}
	}
	HAL_UARTEx_ReceiveToIdle_IT(handle, recvbuffer, sizeof(recvbuffer));
}

void dsmr_test() {
	char test[] =
			"/XMX5LGBBFG1009089654\r\n"
					"\r\n"
					"1-3:0.2.8(42)\r\n"
					"0-0:1.0.0(250302131650W)\r\n"
					"0-0:96.1.1(4530303330303033313131383833393135)\r\n"
					"1-0:1.8.1(008731.603*kWh)\r\n"
					"1-0:1.8.2(007512.111*kWh)\r\n"
					"1-0:2.8.1(000000.000*kWh)\r\n"
					"1-0:2.8.2(000000.000*kWh)\r\n"
					"0-0:96.14.0(0001)\r\n"
					"1-0:1.7.0(00.238*kW)\r\n"
					"1-0:2.7.0(00.000*kW)\r\n"
					"0-0:96.7.21(00004)\r\n"
					"0-0:96.7.9(00003)\r\n"
					"1-0:99.97.0(3)(0-0:96.7.19)(240224223805W)(0000003993*s)(221230031046W)(0000000250*s)(180521110904S)(0000003882*s)\r\n"
					"1-0:32.32.0(00001)\r\n"
					"1-0:52.32.0(00000)\r\n"
					"1-0:72.32.0(00000)\r\n"
					"1-0:32.36.0(00001)\r\n"
					"1-0:52.36.0(00001)\r\n"
					"1-0:72.36.0(00001)\r\n"
					"0-0:96.13.1()\r\n"
					"0-0:96.13.0()\r\n"
					"1-0:31.7.0(000*A)\r\n"
					"1-0:51.7.0(001*A)\r\n"
					"1-0:71.7.0(000*A)\r\n"
					"1-0:21.7.0(00.033*kW)\r\n"
					"1-0:41.7.0(00.205*kW)\r\n"
					"1-0:61.7.0(00.000*kW)\r\n"
					"1-0:22.7.0(00.000*kW)\r\n"
					"1-0:42.7.0(00.000*kW)\r\n"
					"1-0:62.7.0(00.000*kW)\r\n"
					"!2CAD\r\n";

//	char test[] = "/KFM5KAIFA-METER\r\n"
//			"\r\n"
//			"1-3:0.2.8(42)\r\n"
//			"0-0:1.0.0(231010165231S)\r\n"
//			"0-0:96.1.1(4530303033303030303035383130313134)\r\n"
//			"1-0:1.8.1(011550.656*kWh)\r\n"
//			"1-0:1.8.2(009865.285*kWh)\r\n"
//			"1-0:2.8.1(000005.892*kWh)\r\n"
//			"1-0:2.8.2(000012.820*kWh)\r\n"
//			"0-0:96.14.0(0002)\r\n"
//			"1-0:1.7.0(00.070*kW)\r\n"
//			"1-0:2.7.0(00.000*kW)\r\n"
//			"0-0:96.7.21(00009)\r\n"
//			"0-0:96.7.9(00006)\r\n"
//			"1-0:99.97.0(3)(0-0:96.7.19)(211005044043S)(0000000666*s)(180617140348S)(0000000603*s)(000101000008W)(2147483647*s)\r\n"
//			"1-0:32.32.0(00000)\r\n"
//			"1-0:52.32.0(00000)\r\n"
//			"1-0:72.32.0(00000)\r\n"
//			"1-0:32.36.0(00000)\r\n"
//			"1-0:52.36.0(00000)\r\n"
//			"1-0:72.36.0(00000)\r\n"
//			"0-0:96.13.1()\r\n"
//			"0-0:96.13.0()\r\n"
//			"1-0:31.7.0(000*A)\r\n"
//			"1-0:51.7.0(000*A)\r\n"
//			"1-0:71.7.0(000*A)\r\n"
//			"1-0:21.7.0(00.034*kW)\r\n"
//			"1-0:22.7.0(00.000*kW)\r\n"
//			"1-0:41.7.0(00.020*kW)\r\n"
//			"1-0:42.7.0(00.000*kW)\r\n"
//			"1-0:61.7.0(00.016*kW)\r\n"
//			"1-0:62.7.0(00.000*kW)\r\n"
//			"0-1:24.1.0(003)\r\n"
//			"0-1:96.1.0(4730303137353931323133333336383134)\r\n"
//			"0-1:24.2.1(231010160000S)(09973.642*m3)\r\n"
//			"!B137\r\n";

	meterstand = dsmr_parse(test, sizeof(test));
}
int main() {
	SEGGER_RTT_Init();

	puts("BlaatSchaap Domotica: DSMR");
	printf("Serial: %s\n", get_serial_string());

	extern void ClockSetup_HSI_SYS48(void); // TODO
	extern void ClockSetup_HSE8_SYS48(void);
	extern void ClockSetup_HSI_SYS8(void);
	extern void ClockSetup_HSI_SYS36(void);
//	ClockSetup_HSI_SYS48();
//	ClockSetup_HSE8_SYS48();
//	ClockSetup_HSI_SYS8();
	ClockSetup_HSI_SYS36();

	HAL_Init(); // gah

	timer_init();

	// Time zone on the microcontroller
	// https://newlib.sourceware.narkive.com/fvlGlRPa/how-to-set-timezone-for-localtime
	// https://stackoverflow.com/questions/73935736/time-zone-format
	putenv("TZ=CET-1CEST,M3.5.0,M10.5.0/3");
	tzset();

	bshal_delay_init();

	gpio_init();

	ir_init();
	i2c_init();

	display_init();

	display_clear();

	display_print_upper("  DSMR  ");
	display_print_middle("Domotica");
	display_print_lower("  Rosi  ");
	display_apply();

	timer_init();
	rtc_init();
	// Testing how long it takes for the Crystalless device
	printf("Time after rtc init %4d ms\n", get_time_ms());
	// Force a delay
	while (get_time_ms() < 3000)
		;

	radio_init(&m_radio);
	gp_radio = &m_radio;
	radio_irq_init();
	bsradio_set_mode(&m_radio, mode_receive);

	protocol_register_command(info_handler, BSCP_CMD_INFO);
	protocol_register_command(unixtime_handler, BSCP_CMD_UNIXTIME);

	protocol_register_command(pair_handler, BSCP_CMD_PAIR);

	dsmr_test();

	uart_init();

	while (1) {

		dsmr_process();
//		sensors_process(); 	//timekeeper("sensors_process");
//		buttons_process(); 	//timekeeper("buttons_process");
		display_process(); 	//timekeeper("display_process");
//		ir_process();   	//timekeeper("ir_process");
		radio_process();	//timekeeper("radio_process");

	}
}

uint32_t get_network_id(void) {
	return *((uint32_t*) m_radio.rfconfig.network_id);
}

uint8_t get_node_id(void) {
	return m_radio.rfconfig.node_id;
}

void enter_normal_mode(void) {
	radio_write_rfconfig();
	bshal_gpio_write_pin(m_radio.spim.rs_pin, 1);
	bshal_delay_ms(5);
	bshal_gpio_write_pin(m_radio.spim.rs_pin, 0);
	bshal_delay_ms(50);
	bsradio_init(&m_radio);
}

void enter_pair_mode(void) {
	static bsradio_instance_t radio_pairmode;
	radio_pairmode = m_radio;
	(*(uint32_t*) radio_pairmode.rfconfig.network_id) = 0xdeadbeef;
	radio_pairmode.rfconfig.node_id = 0xFE;
	bshal_gpio_write_pin(m_radio.spim.rs_pin, 1);
	bshal_delay_ms(5);
	bshal_gpio_write_pin(m_radio.spim.rs_pin, 0);
	bshal_delay_ms(50);
	bsradio_init(&radio_pairmode);
}
