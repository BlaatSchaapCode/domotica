/*
 * test.c
 *
 *  Created on: 19 jan. 2024
 *      Author: andre
 */

// Parking some test code used earlier

int main_() {
  gpio_init();
  bshal_delay_init();
  while (1) {
    if (!(bshal_gpio_read_pin(0))) {
      bshal_gpio_write_pin(1, 0);
      bshal_delay_ms(500);
    } else if (!(bshal_gpio_read_pin(2))) {
      bshal_gpio_write_pin(1, 1);
      bshal_delay_ms(500);
    }
  }
}

void spi_pins_test(void) {
  bshal_gpio_cfg_out(5, pushpull, 0);
  bshal_gpio_cfg_out(6, pushpull, 0);
  bshal_gpio_cfg_out(7, pushpull, 0);
  bshal_gpio_cfg_out(17, pushpull, 0);

  while (1) {
    bshal_gpio_write_pin(6, 1);
    bshal_delay_ms(1);
    bshal_gpio_write_pin(6, 0);
    bshal_delay_ms(1);

    bshal_gpio_write_pin(7, 1);
    bshal_delay_ms(1);
    bshal_gpio_write_pin(7, 0);
    bshal_delay_ms(1);

    bshal_gpio_write_pin(5, 1);
    bshal_delay_ms(1);
    bshal_gpio_write_pin(5, 0);
    bshal_delay_ms(1);

    bshal_gpio_write_pin(17, 1);
    bshal_delay_ms(1);
    bshal_gpio_write_pin(17, 0);
    bshal_delay_ms(1);
  }
}
