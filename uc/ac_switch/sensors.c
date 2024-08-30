/*
 * sensors.c
 *
 *  Created on: 16 dec. 2023
 *      Author: andre
 */

#include "bshal_delay.h"
#include "bshal_i2cm.h"

#include "bh1750.h"
#include "bmp280.h"
#include "ccs811.h"
#include "hcd1080.h"
#include "lm75b.h"
#include "scd4x.h"
#include "sht3x.h"
#include "si70xx.h"

#include "bsradio.h"
#include "sensor_protocol.h"
#include "timer.h"

extern bsradio_instance_t *gp_radio;

#include <stdint.h>
#include <string.h>

lm75b_t lm75b = {};
bh1750_t bh1750 = {};

int16_t lm75b_temperature_centi_celcius;
uint16_t bh1750_illuminance_lux;

bool lm75b_ready = false;
bool bh1750_ready = false;

extern bshal_i2cm_instance_t *gp_i2c;

void sensors_send(void) {
  bsradio_packet_t request = {}, response = {};
  request.from = gp_radio->rfconfig.node_id; // 0x03;
  request.to = 0x00;
#pragma pack(push, 1)
  struct sensor_data_packet {
    bscp_protocol_header_t head;
    bsprot_sensor_enviromental_data_t data;
  } sensor_data_packet;
  bscp_protocol_packet_t *packet =
      (bscp_protocol_packet_t *)(&sensor_data_packet);
  sensor_data_packet.head.size = sizeof(sensor_data_packet);
  sensor_data_packet.head.cmd = BSCP_CMD_SENSOR_ENVIOREMENTAL_VALUE;
  sensor_data_packet.head.sub = BSCP_SUB_SDAT;
#pragma pack(pop)
  if (lm75b_ready) {
    lm75b_ready = false;

    sensor_data_packet.data.id = 0;
    sensor_data_packet.data.type = bsprot_sensor_enviromental_temperature;
    sensor_data_packet.data.value.temperature_centi_celcius =
        lm75b_temperature_centi_celcius;
    if (protocol_packet_merge(request.payload, sizeof(request.payload),
                              packet)) {
      // packet is full, send it.
      request.length = 4 + protocol_merged_packet_size(request.payload,
                                                       sizeof(request.payload));
      bsradio_send_request(gp_radio, &request, &response);
      request.payload[0] = 1;
      memset(request.payload, 0, sizeof(request.payload));
      protocol_packet_merge(request.payload, sizeof(request.payload), packet);
    }
  }

  if (bh1750_ready) {
    bh1750_ready = false;

    sensor_data_packet.data.id = 1;
    sensor_data_packet.data.type = bsprot_sensor_enviromental_illuminance;
    sensor_data_packet.data.value.illuminance_lux = bh1750_illuminance_lux;
    if (protocol_packet_merge(request.payload, sizeof(request.payload),
                              packet)) {
      // packet is full, send it.
      request.length = 4 + protocol_merged_packet_size(request.payload,
                                                       sizeof(request.payload));
      bsradio_send_request(gp_radio, &request, &response);
      //			request.payload[0] = 0;
      memset(request.payload, 0, sizeof(request.payload));
      protocol_packet_merge(request.payload, sizeof(request.payload), packet);
    }
  }

  // That's all, send remaining
  request.length =
      4 + protocol_merged_packet_size(request.payload, sizeof(request.payload));
  bsradio_send_request(gp_radio, &request, &response);
  //	request.payload[0] = 0;
  memset(request.payload, 0, sizeof(request.payload));
}

void sensors_process(void) {
  static uint32_t process_time = 0;
  if (process_time > 5000 && ((int)get_time_ms() - (int)process_time < -5000)) {
    // handle overflow
    process_time = 0;
  }
  if (get_time_ms() > process_time) {
    // process_time = get_time_ms() + 5000;
    process_time = get_time_ms() + 1000;

    if (lm75b.addr) {
      //			float temperature_f;
      //			lm75b_get_temperature_C_float(&lm75b,
      //&temperature_f); 			lm75b_temperature_centi_celcius = 100 * temperature_f;

      accum temperature_a;
      lm75b_get_temperature_C_accum(&lm75b, &temperature_a);
      lm75b_temperature_centi_celcius = 100 * temperature_a;

      lm75b_ready = true;
    }

    if (bh1750.addr) {
      static int lux = 0;
      bh1750_measure_ambient_light(&bh1750, &lux);
      bh1750_illuminance_lux = lux;
      bh1750_ready = true;
    }
  }
}

void sensors_init(void) {
  if (0 == bshal_i2cm_isok(gp_i2c, LM75B_I2C_ADDR)) {
    lm75b.addr = LM75B_I2C_ADDR;
    lm75b.p_i2c = gp_i2c;
  }
  if (0 == bshal_i2cm_isok(gp_i2c, BH1750_I2C_ADDR)) {
    bh1750.addr = BH1750_I2C_ADDR;
    bh1750.p_i2c = gp_i2c;
  }
}
