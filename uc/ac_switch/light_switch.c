/*
 * light_switch.c
 *
 *  Created on: 19 jan. 2024
 *      Author: andre
 */

#include "light_switch.h"

#include <string.h>

#include "protocol.h"
#include "switch_protocol.h"
#include "bsradio.h"

#include <bshal_gpio.h>

extern bsradio_instance_t *gp_radio;

void light_switch_send(void) {
  bsradio_packet_t request = {}, response = {};
  request.from = gp_radio->rfconfig.node_id; // 0x03;
  request.to = 0x00;
#pragma pack(push, 1)
  struct sensor_data_packet {
    bscp_protocol_header_t head;
    uint8_t switch_state;
  } switch_data_packet;
  switch_data_packet.switch_state = light_switch_get();
  switch_data_packet.head.size = sizeof(switch_data_packet);
  switch_data_packet.head.cmd = BSCP_CMD_SWITCH;
  switch_data_packet.head.sub = BSCP_SUB_SDAT;
  request.length = 4 + switch_data_packet.head.size;
  memcpy(request.payload, &switch_data_packet, switch_data_packet.head.size);
  bsradio_send_request(gp_radio, &request, &response);
}
void light_switch_set(bool value) {
	bshal_gpio_write_pin(1, value);
	light_switch_send();

}
bool light_switch_get(void) { return bshal_gpio_read_pin(1); }
bool button1_get(void) { return !bshal_gpio_read_pin(0); }
bool button2_get(void) { return !bshal_gpio_read_pin(2); }
