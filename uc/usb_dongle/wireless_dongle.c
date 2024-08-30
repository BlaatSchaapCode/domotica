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

#include "bshal_delay.h"
#include "bshal_i2cm.h"
#include "bshal_spim.h"

// #include "lm75b.h"
// #include "sht3x.h"
// #include "bh1750.h"
// #include "hcd1080.h"
// #include "si70xx.h"
// #include "ccs811.h"
// #include "bmp280.h"
// #include "scd4x.h"

#include "si4x3x.h"
#include "si4x6x.h"
#include "sxv1.h"

#include "protocol.h"
#include "sensor_protocol.h"

#include "SEGGER_RTT.h"
#include "bshal_gpio_stm32.h"
#include "bshal_i2cm.h"
#include "bshal_i2cm_stm32.h"
#include "spi_flash.h"
#include "timer.h"
#include "usbd.h"
#include <stdio.h>

static bshal_i2cm_instance_t m_i2c;
bshal_spim_instance_t spi_flash_config;
bsradio_instance_t m_radio;
bscp_usbd_handle_t *gp_usbd;

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

void spi_flash_init(void) {
  spi_flash_config.frequency = 1000000;
  spi_flash_config.bit_order = 0; // MSB
  spi_flash_config.mode = 0;

  spi_flash_config.hw_nr = 1;
  spi_flash_config.sck_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_5);
  spi_flash_config.miso_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_6);
  spi_flash_config.mosi_pin = bshal_gpio_encode_pin(GPIOA, GPIO_PIN_7);
  spi_flash_config.cs_pin = bshal_gpio_encode_pin(GPIOB, GPIO_PIN_1);
  spi_flash_config.rs_pin = bshal_gpio_encode_pin(GPIOB, GPIO_PIN_10);

  bshal_spim_init(&spi_flash_config);
}

int radio_init(bsradio_instance_t *bsradio) {
  spi_flash_init();
  uint8_t hwconfig_buffer[256] = {};
  bscp_protocol_header_t *header = (bscp_protocol_header_t *)(hwconfig_buffer);
  spi_flash_read(&spi_flash_config, 0x000, hwconfig_buffer,
                 sizeof(hwconfig_buffer));
  if (header->size ==
      sizeof(bscp_protocol_header_t) + sizeof(bsradio_hwconfig_t)) {
    // Should check the whole header, but for testing keep it like this
    //		header->cmd == 0x02;
    //		header->sub == 0x20;
    //		header->res == 'R';
    bsradio_hwconfig_t *hwconfig =
        (bsradio_hwconfig_t *)(hwconfig_buffer +
                               sizeof(bscp_protocol_header_t));
    bsradio->hwconfig = *hwconfig;
    puts("hwconfig loaded");
  } else {
    puts("hwconfig missing");
    return -1;
  }

  // Need to write this first
  uint8_t rfconfig_buffer[256] = {};
  header = (bscp_protocol_header_t *)(rfconfig_buffer);
  spi_flash_read(&spi_flash_config, 0x100, rfconfig_buffer,
                 sizeof(rfconfig_buffer));

  // temp disable
  if (false && header->size == sizeof(bscp_protocol_header_t) +
                                   sizeof(bsradio_rfconfig_t)) {
    bsradio_rfconfig_t *rfconfig =
        (bsradio_rfconfig_t *)rfconfig_buffer + sizeof(bscp_protocol_header_t);
    bsradio->rfconfig = *rfconfig;
    puts("rfconfig loaded");
  } else {
    puts("rfconfig missing");

    switch (bsradio->hwconfig.frequency_band) {
    case 434:
      //		bsradio->driver.set_frequency(bsradio, 434000);
      //		bsradio->driver.set_tx_power(bsradio, 10);
      bsradio->rfconfig.frequency_kHz = 434000;
      bsradio->rfconfig.tx_power_dBm = 10;

      break;
    case 868:
      bsradio->rfconfig.frequency_kHz = 869850;
      //		bsradio->rfconfig.frequency_kHz = 870000;
      bsradio->rfconfig.tx_power_dBm = 0;
      break;
    case 915:
      // Sorry Americans... your FCC only allows very weak signals
      //		bsradio->driver.set_frequency(bsradio, 915000);
      //		bsradio->driver.set_tx_power(bsradio, -3);
      bsradio->rfconfig.frequency_kHz = 915000;
      bsradio->rfconfig.tx_power_dBm = -3;
      break;
    }

    //	bsradio->rfconfig.modulation_shaping = 0;
    bsradio->rfconfig.modulation_shaping = 5; // 0.5 gfsk
    bsradio->rfconfig.modulation = modulation_2fsk;
    // bsradio->rfconfig.modulation = modulation_ook;

    // TODO --> update radio config
    // and do all these calls in the init function
    // sxv1_set_sync_word32(bsradio, 0xdeadbeef);
    //	bsradio->driver.set_bitrate(bsradio, 12500);
    //	bsradio->driver.set_fdev(bsradio, 12500);
    //	bsradio->driver.set_bandwidth(bsradio, 25000);

    //		bsradio->rfconfig.birrate_bps = 12500;
    //		bsradio->rfconfig.freq_dev_hz = 12500;
    //		bsradio->rfconfig.bandwidth_hz = 25000;

    bsradio->rfconfig.birrate_bps = 25000;
    bsradio->rfconfig.freq_dev_hz = 25000;
    bsradio->rfconfig.bandwidth_hz = 50000;

    //		bsradio->rfconfig.birrate_bps = 50000;
    //		bsradio->rfconfig.freq_dev_hz = 50000;
    //		bsradio->rfconfig.bandwidth_hz = 100000;

    bsradio->rfconfig.network_id[0] = 0xDE;
    bsradio->rfconfig.network_id[1] = 0xAD;
    bsradio->rfconfig.network_id[2] = 0xBE;
    bsradio->rfconfig.network_id[3] = 0xEF;
    bsradio->rfconfig.network_id_size = 4;

    bsradio->rfconfig.node_id = 0x00;
    bsradio->rfconfig.broadcast_id = 0xFF;

    bool update_flash = false;
    if (update_flash) {
      uint8_t buffer[256];
      bscp_protocol_header_t *header = (bscp_protocol_header_t *)buffer;
      bsradio_rfconfig_t *rfconfig =
          (bsradio_rfconfig_t *)(buffer + sizeof(bscp_protocol_header_t));
      header->size =
          sizeof(bscp_protocol_header_t) + sizeof(bsradio_rfconfig_t);
      header->cmd = 0x02;
      header->sub = 0x20;
      header->res = 'R';
      *rfconfig = (bsradio->rfconfig);
      puts("Erasing page");
      spi_flash_erase_page_256(&spi_flash_config, 0x100);
      puts("Programming");
      spi_flash_program(&spi_flash_config, 0x100, buffer, header->size);
      puts("Done");
    }
  }

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
  switch (bsradio->hwconfig.chip_brand) {
  case chip_brand_semtech:
    switch (bsradio->hwconfig.chip_type) {
    case 1:
      puts("Semtech variant 1");
      // Semtech variant 1
      // Transceiver: SX1231, SX1231H, SX1233, RFM69,
      // Receiver only MRF39RA: SX1239
      // Potentially others. This variant can be recognised by the
      // presence of register RegVersion at 0x10
      // Tested on RFM69

      // reset is active high!!!!
      // TODO: Can we set reset polarity in our config
      // and reset universally in stead of per chip?
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
      break;
    case 2:
      // Semtech variant 2
      // Transceiver OOK/FSK: : SX1232, SX1236
      // Transceiver OOK/FSK/LoRa: SX127x, RFM9x
      // Potentially others. This variant can be recognised by the
      // presence of register RegVersion at 0x42
      // Npt yet implemented. various modules in possession
      // TODO
      //					bsradio->driver.set_frequency =
      //sxv2_set_frequency; 					bsradio->driver.set_tx_power = sxv2_set_tx_power;
      //					bsradio->driver.set_bitrate =
      //sxv2_set_bitrate; 					bsradio->driver.set_fdev = sxv2_set_fdev;
      //					bsradio->driver.set_bandwidth =
      //sxv2_set_bandwidth; 					bsradio->driver.init = sxv2_init;
      //					bsradio->driver.set_network_id =
      //sxv2_set_network_id; 					bsradio->driver.set_mode = sxv2_set_mode;
      //					bsradio->driver.recv_packet =
      //sxv2_recv_packet; 					bsradio->driver.send_packet = sxv2_send_packet;
      break;
    case 3:
      // Semtech variant 3
      // LLCC68, SX1261, SX1262, SX1268
      // No modules in possession

      //					bsradio->driver.set_frequency =
      //sxv3_set_frequency; 					bsradio->driver.set_tx_power = sxv3_set_tx_power;
      //					bsradio->driver.set_bitrate =
      //sxv3_set_bitrate; 					bsradio->driver.set_fdev = sxv3_set_fdev;
      //					bsradio->driver.set_bandwidth =
      //sxv3_set_bandwidth; 					bsradio->driver.init = sxv3_init;
      //					bsradio->driver.set_network_id =
      //sxv3_set_network_id; 					bsradio->driver.set_mode = sxv3_set_mode;
      //					bsradio->driver.recv_packet =
      //sxv3_recv_packet; 					bsradio->driver.send_packet = sxv3_send_packet;
    default:
      return -1;
      break;
    }
    break;
  case chip_brand_silabs:
    switch (bsradio->hwconfig.chip_type) {
    case 1:
      // SiLabs Si4x3x chips.
      // These are well documented, unfortunately not recommended for new design
      //					bsradio->driver.set_frequency =
      //si4x3x_set_frequency; 					bsradio->driver.set_tx_power =
      //si4x3x_set_tx_power; 					bsradio->driver.set_bitrate = si4x3x_set_bitrate;
      //					bsradio->driver.set_fdev =
      //si4x3x_set_fdev; 					bsradio->driver.set_bandwidth = si4x3x_set_bandwidth;
      //					bsradio->driver.init =
      //si4x3x_init; 					bsradio->driver.set_network_id = si4x3x_set_network_id;
      //					bsradio->driver.set_mode =
      //si4x3x_set_mode; 					bsradio->driver.recv_packet = si4x3x_recv_packet;
      //					bsradio->driver.send_packet =
      //si4x3x_send_packet;
      break;
    case 2:
      // SiLabs Si4x6x chips.
      // This is the newer generation of SiLabs Radio chips
      // While there is extended documentation, many details are missing.
      // The official development way is getting magic values generated
      // by a tool that only runs on Microsoft Windows.
      // This tool takes all the frequency, deviation, bandwidth, packet format
      // all the settings and generates a header file to use.
      // As this project wishes to create an API to configure the radio
      // parameters, this is going to be a trickier one to support.
      //
      // TODO
      //					bsradio->driver.set_frequency =
      //si4x6x_set_frequency; 					bsradio->driver.set_tx_power =
      //si4x6x_set_tx_power; 					bsradio->driver.set_bitrate = si4x6x_set_bitrate;
      //					bsradio->driver.set_fdev =
      //si4x6x_set_fdev; 					bsradio->driver.set_bandwidth = si4x6x_set_bandwidth;
      //					bsradio->driver.init =
      //si4x6x_init; 					bsradio->driver.set_network_id = si4x6x_set_network_id;
      //					bsradio->driver.set_mode =
      //si4x6x_set_mode; 					bsradio->driver.recv_packet = si4x6x_recv_packet;
      //					bsradio->driver.send_packet =
      //si4x6x_send_packet;
      break;
    default:
      return -1;
      break;
    }
    break;

  default:
    return -1;
    break;
  }

  /*
   * Europe: 433 MHz Band:
   * 433.050 MHz to 434.790 MHz:
   * +10 dBm if bandwidth ≤ 25 kHz or duty cycle ≤ 10%
   * otherwise 0 dBm

   * Europe: 868 MHz band
   * 863.000 to 870.000 MHz: +14 dBm
   *		863,00 MHz to 865,00 MHz	≤ 0,1 % duty cycle
   *		865,00 MHz to 868,00 MHz	≤ 1,0 % duty cycle
   * 		868,00 MHz to 868,60 MHz	≤ 1,0 % duty cycle
   * 		868,70 MHz to 869,20 MHz	≤ 0,1 % duty cycle
   * 		869,40 MHz to 869,65 MHz	≤ 0.1 % duty cycle
   * 		869,70 MHz to 870,00 MHz	≤ 1,0 % duty cycle
   *
   * 		869,40 MHz to 869,65 MHz:	500 mW = +27 dBm	≤ 10 %
   duty cycle
   * 		869,70 MHz to 870,00 MHz:	  5 mW =  +7 dBm	 no duty
   cycle limit
   *
   * 		For any duty cycle limit, it says "or polite spectrum access"
   * 		Not sure about the definition though.
   * 		The dBm is "ERP"
   *
   *
   *
   * North America: 915 MHz band:
   * 902 to 928 MHz:
   * Single Frequency:	–1.23 dBm
   * Frequency Hopping:	+24 dBm
   * 		The dBm is "EIRP",
   * 		ERP = EIRP - 2.15 dB
   *
   *
   * How to these ERP or EIRP values relate to the settings I give to the
   module?
   * What is the loss of the antenna? What is the directional gain?
   * In other words, what am I allowed to set?
   */

  extern uint32_t get_serial(void);
  (*(uint32_t *)bsradio->rfconfig.network_id) = get_serial();
  bsradio->rfconfig.network_id_size = 4;
  bsradio->rfconfig.node_id = 0;

  return bsradio_init(bsradio);
}

bscp_handler_status_t forward_handler(bscp_protocol_packet_t *packet,
                                      protocol_transport_t transport,
                                      uint32_t param) {
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
        usb_response.head = packet->head;
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
                                         uint32_t param) {
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

  // New GCC versions are nit-picking.
  // Put them here for now, make it neat later.
  extern void ClockSetup_HSE8_SYS72(void);
  extern void usbd_reenumerate(void);
  extern void usbd_process(void);

  ClockSetup_HSE8_SYS72();
  SystemCoreClockUpdate();
  SEGGER_RTT_Init();
  puts("Wireless Dongle");
  usbd_reenumerate();
  gp_usbd = usbd_init();
  timer_init();
  i2c_init();
  radio_init(&m_radio);
  bsradio_set_mode(&m_radio, mode_receive);

  protocol_register_command(sensordata_handler,
                            BSCP_CMD_SENSOR_ENVIOREMENTAL_VALUE);

  protocol_register_command(forward_handler, BSCP_CMD_FORWARD);
  while (1) {

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
      forward_packet->head.sub = BSCP_SUB_SDAT;

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
