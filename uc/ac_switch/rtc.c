#include "stm32f1xx_hal.h"

RTC_HandleTypeDef RtcHandle;

#include <time.h>
#include <stdbool.h>
#include <stdio.h>

void RTC_IRQHandler(void) {
	HAL_RTCEx_RTCIRQHandler(&RtcHandle);
}

void Error_Handler(void) {
	//	puts("Error!");
}

/**
 * @brief  Enters the RTC Initialization mode.
 * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @retval HAL status
 */
static HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *hrtc) {
	uint32_t tickstart = 0U;

	tickstart = HAL_GetTick();
	/* Wait till RTC is in INIT state and if Time out is reached exit */
	while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t) RESET) {
		if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE) {
			return HAL_TIMEOUT;
		}
	}

	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_DISABLE(hrtc);

	return HAL_OK;
}

/**
 * @brief  Exit the RTC Initialization mode.
 * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @retval HAL status
 */
static HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef *hrtc) {
	uint32_t tickstart = 0U;

	/* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_ENABLE(hrtc);

	tickstart = HAL_GetTick();
	/* Wait till RTC is in INIT state and if Time out is reached exit */
	while ((hrtc->Instance->CRL & RTC_CRL_RTOFF) == (uint32_t) RESET) {
		if ((HAL_GetTick() - tickstart) > RTC_TIMEOUT_VALUE) {
			return HAL_TIMEOUT;
		}
	}

	return HAL_OK;
}

static uint32_t RTC_ReadTimeCounter(RTC_HandleTypeDef *hrtc) {
	uint16_t high1 = 0U, high2 = 0U, low = 0U;
	uint32_t timecounter = 0U;

	high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
	low = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
	high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

	if (high1 != high2) {
		/* In this case the counter roll over during reading of CNTL and CNTH
		 registers, read again CNTL register then return the counter value */
		timecounter = (((uint32_t) high2 << 16U)
				| READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
	} else {
		/* No counter roll over during reading of CNTL and CNTH registers, counter
		 value is equal to first value of CNTL and CNTH */
		timecounter = (((uint32_t) high1 << 16U) | low);
	}

	return timecounter;
}

/**
 * @brief  Write the time counter in RTC_CNT registers.
 * @param  hrtc   pointer to a RTC_HandleTypeDef structure that contains
 *                the configuration information for RTC.
 * @param  TimeCounter: Counter to write in RTC_CNT registers
 * @retval HAL status
 */
static HAL_StatusTypeDef RTC_WriteTimeCounter(RTC_HandleTypeDef *hrtc,
		uint32_t TimeCounter) {
	HAL_StatusTypeDef status = HAL_OK;

	/* Set Initialization mode */
	if (RTC_EnterInitMode(hrtc) != HAL_OK) {
		status = HAL_ERROR;
	} else {
		/* Set RTC COUNTER MSB word */
		WRITE_REG(hrtc->Instance->CNTH, (TimeCounter >> 16U));
		/* Set RTC COUNTER LSB word */
		WRITE_REG(hrtc->Instance->CNTL, (TimeCounter & RTC_CNTL_RTC_CNT));

		/* Wait for synchro */
		if (RTC_ExitInitMode(hrtc) != HAL_OK) {
			status = HAL_ERROR;
		}
	}

	return status;
}

time_t time(time_t *t) {
	time_t result = RTC_ReadTimeCounter(&RtcHandle);
	if (t)
		*t = result;
	return result;
}

void time_set(time_t new_time) {
	RTC_WriteTimeCounter(&RtcHandle, new_time);
}

void rtc_init(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/*##-1- Configure LSI as RTC clock source
	 * ###################################*/
	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

	bool crystal = false;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK) {
		puts("Low Speed Crystal Present");
		crystal = true;
	} else {
		puts("Low Speed Crystal Absent");
		crystal = false;
		RCC_OscInitStruct.LSIState = RCC_LSI_ON;
		RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
		if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK) {
			puts("Configured for LSI");
		}

	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	if (crystal)
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	else
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/*##-2- Enable RTC peripheral Clocks #######################################*/
	/* Enable RTC Clock */
	__HAL_RCC_RTC_ENABLE();

	/*##-1- Configure the RTC peripheral #######################################*/
	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follow:
	 - Asynch Prediv  = Calculated automatically by HAL (based on LSI at 40kHz) */
	RtcHandle.Instance = RTC;
	if (crystal)
		RtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	else
		RtcHandle.Init.AsynchPrediv = 43333;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	if (time(NULL) < 1700000000) {
		time_set(1700000000);
	}

	return;
}
