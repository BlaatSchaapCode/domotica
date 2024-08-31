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
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&m_u8g2, U8G2_R0,
			bshal_u8x8_byte_i2c, bshal_u8x8_gpio_and_delay);
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

void display_apply() {
	u8g2_UpdateDisplay(&m_u8g2);
}

void display_clear() {
	u8g2_ClearBuffer(&m_u8g2);
}

void display_process(void) {
	char buff[16];
	//	static time_t prev_time = 0;
	static int state = 1;
	bool light_status = light_switch_get();
	static bool display_status = true;
	static int addrentry[3];

	memset(buff, 0x20, 8);

	switch (m_key) {
	case '*': {
		// Toggle Light
		light_status = !light_status;
		light_switch_set(light_status);
	}
		break;

	case '#': {
		// Display Sensors
		display_status = !display_status;
		u8g2_SetPowerSave(&m_u8g2, !display_status);
		state = display_status;
	}
		break;


	case 'K': {
		u8g2_SetPowerSave(&m_u8g2, false);
		if (state == 0x01) {
			state = 0x10;
		} else if (state == 0x10) {
			state = 0x01;
		}
	}
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
		display_print_upper ("  pair");
		display_print_middle("  mode");
		break;
	default:
		state = 0;
	}
	display_apply();
}

void display_set_key(char key) {
	m_key = key;
}
