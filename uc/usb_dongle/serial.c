/*
 * serial.c
 *
 *  Created on: 28 aug. 2024
 *      Author: andre
 */

#include <stdint.h>
#include <stdio.h>

uint32_t get_serial(void) {
  uint32_t *hwserial = (uint32_t *)(0x1FFFF7E8);
  return hwserial[0] ^ hwserial[1] ^ hwserial[2];
}

char *get_serial_string(void) {
  static char serial[9];
  snprintf(serial, 9, "%08lX", get_serial());
  return serial;
}
