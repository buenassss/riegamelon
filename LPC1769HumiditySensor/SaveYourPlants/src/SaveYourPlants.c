/*
 * @brief FreeRTOS Blinky example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <board.h>
#include <FreeRTOS.h>
#include <task.h>

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define _ADC_CHANNLE ADC_CH0
static ADC_CLOCK_SETUP_T ADCSetup;
#define _LPC_ADC_ID LPC_ADC
#define _LPC_ADC_IRQ ADC_IRQn

static uint16_t humiditySensors[8] = {0};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);

	/*ADC Init */
	Chip_ADC_Init(_LPC_ADC_ID, &ADCSetup);
	Chip_ADC_EnableChannel(_LPC_ADC_ID, _ADC_CHANNLE, ENABLE);
	ADCSetup.burstMode = 1;

	uint32_t _bitRate = ADC_MAX_SAMPLE_RATE;
	Chip_ADC_SetSampleRate(_LPC_ADC_ID, &ADCSetup, _bitRate);
}

/* LED1 toggle thread */
static void vLEDTask1(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(0, LedState);
		LedState = (bool) !LedState;

		/* About a 3Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 2);
	}
}

/* LED2 toggle thread */
static void vSensorTask1(void *pvParameters) {
	uint16_t dataADC;

	/* Select using burst mode or not */
	Chip_ADC_SetBurstCmd(_LPC_ADC_ID, ENABLE);

	/* Get  adc value until get 'x' character */
	while (1) {
		/* Waiting for A/D conversion complete */
		while (Chip_ADC_ReadStatus(_LPC_ADC_ID, _ADC_CHANNLE, ADC_DR_DONE_STAT) != SET) {}
		/* Read ADC value */
		Chip_ADC_ReadValue(_LPC_ADC_ID, _ADC_CHANNLE, &dataADC);
		/* Print ADC value */
		DEBUGOUT("ADC value is : %d\r\n", dataADC);
		humiditySensors[0] = dataADC;

		/* About a 7Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ);
	}

	Chip_ADC_SetBurstCmd(_LPC_ADC_ID, DISABLE);

}

/* UART (or output) thread */
static void whenToWaterTask(void *pvParameters) {

	while (1) {
		//DEBUGOUT("Tick: %d\r\n", tickCnt);
		//tickCnt++;

		for(int i = 0; i < 8; ++i)
		{
			if (humiditySensors[i] > 3000)
			{
				DEBUGOUT("Water plant %d\n", i);
			}
		}

		/* About a 1s delay here */
		vTaskDelay(configTICK_RATE_HZ);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	DEBUGOUT("Setting tasks");
	/* LED1 toggle thread */
	xTaskCreate(vLEDTask1, (signed char *) "vTaskLed1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* LED2 toggle thread */
	xTaskCreate(vSensorTask1, (signed char *) "vSensorTask1",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* UART output thread, simply counts seconds */
	xTaskCreate(whenToWaterTask, (signed char *) "vTaskUart",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}

/**
 * @}
 */
