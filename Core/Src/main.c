/******************************************************************************
 * @file main.c
 * @brief Main program body
 *
 * \todo complete description
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
#include "semphr.h"
// Project
#include "main.h"
#include "my_tests.h"
#include "hcsr04.h"



/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Tasks definitions */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Semaphores and queues definitions */


/* hcsr04 */
extern osThreadId_t sensUltrason_TaskHandle;
extern const osThreadAttr_t sensUltrason_taskAttr;
extern xSemaphoreHandle hcsr04_binSemaph_run_handler;
extern StaticSemaphore_t hcsr04_binSemaph_run_buffer;
extern xQueueHandle hcsr04_queue_handler[NB_HCSR04];

/*******************************************************************************
 * FUNCTIONS PROTOTYPES
 ******************************************************************************/

void StartDefaultTask(void *argument);

/*******************************************************************************
 * MAIN FUNCTION
 * @brief  The application entry point.
 * @retval int
 ******************************************************************************/

int main(void) {

	/* Init HAL & hardware */
	systools_hw_init();
	hcsr04_TIM10_init();

	/* Init scheduler */
	osKernelInitialize();

	/* Creation of tasks */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,	&defaultTask_attributes);
	sensUltrason_TaskHandle = osThreadNew(hcsr04_Task, NULL, &sensUltrason_taskAttr);

	/* Semaphores */
	hcsr04_semaph_queues_init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	while (1) {
		;
	}
}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS
 ******************************************************************************/

/*****************************************
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 ****************************************/

void StartDefaultTask(void *argument) {
	char str_out[40] = "Hello from StartDefaultTask!\n\r";
	systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
	while(1) {
//		vTaskSuspend(NULL);
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);	// Toggle pin state
		osDelay(1000*configTICK_RATE_HZ/1000);

		for (unsigned int hcsr04_index=0; hcsr04_index<NB_HCSR04; hcsr04_index++) {
			int hcsr04_data;
			if ( xQueueReceive(hcsr04_queue_handler[hcsr04_index],  &hcsr04_data, 0) == pdTRUE ) {
				if (hcsr04_data != -1) {
					sprintf(str_out, "hcsr04 %d: %d\n\r", hcsr04_index, hcsr04_data);
				} else {
					sprintf(str_out, "hcsr04 %d: sensor not available\n\r", hcsr04_index, hcsr04_data);
				}
			} else {
				sprintf(str_out, "hcsr04 %d: no new data received\n\r", hcsr04_index);
			}
			systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);

		}

//		test_ts_diff_time();

	}
}

/*****************************************
 * @brief
 * @param  GPIO_Pin:
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


