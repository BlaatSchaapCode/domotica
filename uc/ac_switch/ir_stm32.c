#include <stdbool.h>
#include <stdint.h>

#include "ir.h"
#include "timer.h"
#include <system.h>

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  ir_handler(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3));
}

void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }

void ir_init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
