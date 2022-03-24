/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file main.c
 * @version v1.0
 * @brief entry point for the main application of the robocar system. Based on
 * FreeRTOS, it consists on several tasks:
 * - Task1: sensUltrason. Takes charge of reading the two ultrasonics
 * on board sensors (left and right) to collect information about nearby
 * obsctacles
 * - Task2: communEspSpi. Establishes communication with the control app
 * reporting information on the current status and attending incoming commands.
 * This communication is first solved via SPI to a secondary board which
 * allows wireless communication (Wifi) with the control app.
 * - @todo Task3: XXXXXXXXXXXXX. motor driver (control and encoders)
 * - @todo Task4: XXXXXXXXXXXXX. debugTask, print data out via serial port
 ******************************************************************************/


/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

// Std
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
// Freertos
#include "cmsis_os.h"
#include "queue.h"
#include "semphr.h"
// Project
#include "main.h"
#include "my_tests.h"
#include "hcsr04.h"
#include "communEspSpi.h"
#include "dataCenter.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Tasks definitions */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128*8,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Semaphores and queues definitions */

/* hcsr04 */
extern osThreadId_t sensUltrason_taskHandle;
extern const osThreadAttr_t sensUltrason_taskAttr;
extern xSemaphoreHandle hcsr04_binSemaph_run_handler;
extern xQueueHandle hcsr04_queue_handler[NB_HCSR04];

/* communEspSpi */
extern osThreadId_t communEspSpi_taskHandle;
extern const osThreadAttr_t communEspSpi_taskAttr;
extern xQueueHandle communespspi_in_queue_handler;
extern xQueueHandle communespspi_out_queue_handler;

/* dataCenter */
extern osThreadId_t dataCenter_taskHandle;
extern const osThreadAttr_t dataCenter_taskAttr;

/*******************************************************************************
 * FUNCTIONS PROTOTYPES
 ******************************************************************************/

void StartDefaultTask(void *argument);

/*******************************************************************************
 * FUNCTIONS DEFINITIONS
 ******************************************************************************/

/*******************************************************************************
 * @brief  Application entry point
 * @retval int
 ******************************************************************************/
int main(void) {

	/* Init HAL & hardware */
	systools_hw_init();
	hcsr04_TIM10_init();
	communEspSpi_init();

	/* Init scheduler */
	osKernelInitialize();

	/* Creation of tasks */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,	&defaultTask_attributes);
	sensUltrason_taskHandle = osThreadNew(hcsr04_task, NULL, &sensUltrason_taskAttr);
	communEspSpi_taskHandle = osThreadNew(communEspSpi_task, NULL, &communEspSpi_taskAttr);
	dataCenter_taskHandle = osThreadNew(dataCenter_task, NULL, &dataCenter_taskAttr);

	/* Semaphores */
	hcsr04_semaph_queues_init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1) {
		;
	}
}

/*****************************************
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 ****************************************/

void StartDefaultTask(void *argument) {
	/*
	char str_out[40] = "Hello from StartDefaultTask!\n\r";
	systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
	 */
	while(1) {
/*
//		vTaskSuspend(NULL);
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);	// Toggle pin state

//		test_ts_diff_time();
*/
		osDelay(100*configTICK_RATE_HZ/1000);
	}
}

/*****************************************
 * @brief general interrupt routine for GPIO
 * @param  GPIO_Pin indicates the pin id where the interrupt occurred.
 * @retval None
 ****************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == HCSR04_0_ECHO_PIN) {
		hcsr04_ISR(0);
	} else if (GPIO_Pin == HCSR04_1_ECHO_PIN) {
		hcsr04_ISR(1);
	} else if (GPIO_Pin == USER_Btn_Pin) {
		// If The INT Source USER_button Pin
		static uint32_t tprev = 0;
		static bool first_event = true;
#define T_BOUNCE	250	// in ms
		uint32_t tcurr = osKernelGetTickCount();
		if (systools_diff_time(tprev, tcurr, sizeof(osKernelGetTickCount()))
				> (T_BOUNCE * configTICK_RATE_HZ / 1000) || first_event) {
			tprev = tcurr;
			first_event = false;
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR(hcsr04_binSemaph_run_handler, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}


