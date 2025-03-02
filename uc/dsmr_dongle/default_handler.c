/*
 * default_handler.c
 *
 *  Created on: 22 dec. 2023
 *      Author: andre
 */

#include <system.h>

void Default_Handler(void) {
  int irq = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) - 16;

  printf("Interrupt %d\n", irq);
  __BKPT(0);
}
