/******************************************************************************
 * @file main.c
 * @brief Main program body
 *
 * \todo complete description
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "main.h"
#include "tools_system.h"
#include "cmsis_os.h"

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

/*******************************************************************************
 * FUNCTIONS PROTOTYPES
 ******************************************************************************/




/*******************************************************************************
 * MAIN FUNCTION
 * @brief  The application entry point.
 * @retval int
 ******************************************************************************/

int main(void) {
	/* Enable ICache and DCache */
	SCB_EnableICache();
	SCB_EnableDCache();

	/* Reset of all peripherals, Initializes Flash interface and Systick */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();

	/* Init scheduler */
	osKernelInitialize();

	/* Creation of tasks */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

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
	for (;;) {
		char str_out[40] = "Hello Shorky!\n\r";
		MX_USART3_UART_Transmit((uint8_t *)str_out, 40, 10);
		osDelay(1000);
	}
}


