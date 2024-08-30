#include "system.h"

#include "bshal_i2cm.h"
#include "u8g2.h"
#include "u8x8_i2c.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "bsradio.h"
#include "light_switch.h"
extern bsradio_instance_t *gp_radio;

static u8g2_t m_u8g2;
extern bshal_i2cm_instance_t *gp_i2c;
static char m_key;

void display_init(void) {
  u8g2_SetUserPtr(&m_u8g2, gp_i2c);
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&m_u8g2, U8G2_R0, bshal_u8x8_byte_i2c,
                                         bshal_u8x8_gpio_and_delay);
  u8g2_InitDisplay(&m_u8g2);
  u8g2_SetPowerSave(&m_u8g2, 0);
  u8g2_ClearBuffer(&m_u8g2);
}

void display_print_upper(char *str) {
  // u8g2_SetFont(&m_u8g2, u8g2_font_unifont_t_symbols); //square, with symbols
  // like ⏴ ⏵ ⏶ ⏷
  u8g2_SetFont(&m_u8g2, u8g2_font_inr16_mf);
  u8g2_DrawUTF8(&m_u8g2, 8, 16, str);
}

void display_print_middle(char *str) {
  // u8g2_SetFont(&m_u8g2, u8g2_font_inr33_mr);
  u8g2_SetFont(&m_u8g2, u8g2_font_inr16_mf);
  u8g2_DrawUTF8(&m_u8g2, 8, 40, str);
}

void display_print_lower(char *str) {
  // u8g2_SetFont(&m_u8g2, u8g2_font_inr33_mr);
  u8g2_SetFont(&m_u8g2, u8g2_font_inr16_mf);
  u8g2_DrawUTF8(&m_u8g2, 8, 64, str);
}

void display_print_large(char *str) {
  u8g2_SetFont(&m_u8g2, u8g2_font_inr33_mr);
  //	u8g2_SetFont(&m_u8g2, u8g2_font_inr16_mr);
  u8g2_DrawUTF8(&m_u8g2, 0, 56, str);
}

void display_apply() { u8g2_UpdateDisplay(&m_u8g2); }

void display_clear() { u8g2_ClearBuffer(&m_u8g2); }

void display_process(void) {
  char buff[16];
  //	static time_t prev_time = 0;
  static int state = 1;
  static bool light_status = false;
  static bool display_status = true;
  static int addrentry[3];

  memset(buff, 0x20, 8);

  switch (m_key) {
  case '*': {
    // Toggle Light
    light_status = !light_status;
    light_switch_set(light_status);
  } break;

  case '#': {
    // Display Sensors
    display_status = !display_status;
    u8g2_SetPowerSave(&m_u8g2, !display_status);
    state = display_status;
  } break;
  case 'K': {

    switch (state) {
    case 0x00:
    case 0x01:
      state = 0x10;
      display_status = true;
      u8g2_SetPowerSave(&m_u8g2, false);
      break;
    case 0x10:
      break;
    case 0x20:
      state = 0x21;
      break;
    case 0x24: {
      int addr = addrentry[0] * 100 + addrentry[1] * 10 + addrentry[2];
      if (addr == 0 || addr > 254) {
        state = 0x2F;
        break;
      } else {
        state = 0x25;
        //				gp_radio->rfconfig.node_id = addr;
        break;
      }
    }

    break;
    }
    break;
  }
  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':
    switch (state) {
    case 0x21:
      state++;
      addrentry[0] = m_key - '0';
      break;
    case 0x22:
      state++;
      addrentry[1] = m_key - '0';
      break;
    case 0x23:
      state++;
      addrentry[2] = m_key - '0';
      break;
    }

    if (state == 0x10 && m_key == '1')
      state = 0x20;
    break;
  }
  display_print_large(buff);
  m_key = 0;

  display_clear();

  switch (state) {
  case 0:
    return;
  case 1:

    if (light_status) {
      display_print_upper("  aan");
    } else {
      display_print_upper("  uit");
    }

    extern uint16_t bh1750_illuminance_lux;
    extern int16_t lm75b_temperature_centi_celcius;

    sprintf(buff, "%5d lx", bh1750_illuminance_lux);
    display_print_middle(buff);
    sprintf(buff, "%3d.%u °C", lm75b_temperature_centi_celcius / 100,
            (abs(lm75b_temperature_centi_celcius) % 100) / 10);
    display_print_lower(buff);
    break;
  case 0x10:
    display_print_upper("  menu");
    display_print_middle("1 adres");
    break;
  case 0x20:
    display_print_upper(" adres");
    snprintf(buff, sizeof buff, "%3d dec", gp_radio->rfconfig.node_id);
    display_print_middle(buff);
    snprintf(buff, sizeof buff, " %02X hex", gp_radio->rfconfig.node_id);
    display_print_lower(buff);
    break;
  case 0x25: {
    int addr = addrentry[0] * 100 + addrentry[1] * 10 + addrentry[2];
    display_print_upper("opslaan?");
    snprintf(buff, sizeof buff, "%3d dec", addr);
    display_print_middle(buff);
    snprintf(buff, sizeof buff, " %02X hex", addr);
    display_print_lower(buff);
  } break;
  case 0x2F:
    display_print_upper("foutief");
    display_print_middle("adres");
    display_print_lower("1-254");
    break;
  case 0x21:
    display_print_upper(" nieuw");
    buff[0] = buff[1] = ' ';
    buff[2] = buff[3] = buff[4] = '_';
    buff[5] = 0;
    display_print_middle(buff);
    break;
  case 0x22:
    display_print_upper(" nieuw");
    buff[0] = buff[1] = ' ';
    buff[2] = buff[3] = buff[4] = '_';
    buff[5] = 0;
    buff[2] = addrentry[0] + '0';
    display_print_middle(buff);
    break;

  case 0x23:
    display_print_upper(" nieuw");
    buff[0] = buff[1] = ' ';
    buff[2] = buff[3] = buff[4] = '_';
    buff[5] = 0;
    buff[2] = addrentry[0] + '0';
    buff[3] = addrentry[1] + '0';
    display_print_middle(buff);
    break;
  case 0x24:
    display_print_upper(" nieuw");
    buff[0] = buff[1] = ' ';
    buff[2] = buff[3] = buff[4] = '_';
    buff[5] = 0;
    buff[2] = addrentry[0] + '0';
    buff[3] = addrentry[1] + '0';
    buff[4] = addrentry[2] + '0';
    display_print_middle(buff);
    break;

    break;
  }
  display_apply();
}

void display_set_key(char key) { m_key = key; }
