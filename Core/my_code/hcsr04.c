/******************************************************************************
 * @file hcsr04.c
 * @brief File containing functions related to hcsr04 ultrasonics sensor
 *
 * One or more hcsr04 ultrasonics sensors may be handled by using this driver. After
 * preparing the requirements, hcsr04_Task will collect data for each sensor and will
 * send the distance in mm gathered in an specific queue per sensor.
 *
 * Warning!! Do not use TIM10 from any other part of the code!
 *
 * Platform: STM Nucleo-F767ZI
 *
 * Requirements:
 * 	1) Define one hcsr04 object per each sensor being used and add it to hcsr04_ptr_array
 * 	2) Per each pin, enable its port clock (__HAL_RCC_GPIOF_CLK_ENABLE)
 * 	3) Initialize gpio: call hcsr04_GPIO_Init()
 * 	4) Initialize timer TIM10: call MX_TIM10_Init()
 * 		Do not use TIM10 from any other part of the code!
 * 	5) Make sure of that NVIC is configured to process Echo pin interrupts
 * 	6) Initialize semaphores and queues: hcsr04_semaph_queues_init()
 * 	7) Initialize task from main: using hcsr04_Task() as task function
 * 	8) Call hcsr04_ISR() from general GPIO ISR when it corresponds
 * 	9) Task will be blocked and will run once each time hcsr04_binSemaph is opened
 *
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "hcsr04.h"
#include "systools.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "queue.h"

/*******************************************************************************
 * DATA STRUCTS
 ******************************************************************************/

typedef struct {
	uint16_t trigger_pin;
	GPIO_TypeDef * trigger_port;
	uint16_t echo_pin;
	GPIO_TypeDef * echo_port;
} hcsr04;

/*******************************************************************************
 * USER
 ******************************************************************************/

/* Sensor 0 */
static const hcsr04 hcsr04_0 = {
		.trigger_pin = HCSR04_0_TRIG_PIN,
		.trigger_port = HCSR04_0_TRIG_PORT,
		.echo_pin = HCSR04_0_ECHO_PIN,
		.echo_port = HCSR04_0_ECHO_PORT,
};
/* Sensor 1 */
static const hcsr04 hcsr04_1 = {
		.trigger_pin = HCSR04_1_TRIG_PIN,
		.trigger_port = HCSR04_1_TRIG_PORT,
		.echo_pin = HCSR04_1_ECHO_PIN,
		.echo_port = HCSR04_1_ECHO_PORT,
};
/* List of sensors */
static const hcsr04* hcsr04_ptr_array[NB_HCSR04] = {
	&hcsr04_0,
	&hcsr04_1,
};

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Timer */
TIM_HandleTypeDef htim10;

/* Semaphores and queues definitions */
xSemaphoreHandle hcsr04_binSemaph_run_handler;
static StaticSemaphore_t hcsr04_binSemaph_run_buffer;

static xSemaphoreHandle hcsr04_binSemaph_echo_handler;
static StaticSemaphore_t hcsr04_binSemaph_echo_buffer;

#define HCSR04_QUEUE_LENGTH	1
xQueueHandle hcsr04_queue_handler[NB_HCSR04];
static StaticQueue_t hcsr04_queue_buffer[NB_HCSR04];
static uint8_t hcsr04_queue_stbuff[NB_HCSR04][HCSR04_QUEUE_LENGTH*sizeof(int)/sizeof(uint8_t)];
				// the queue will store unsigned int elements

/* Task handle / attributes */
osThreadId_t sensUltrason_TaskHandle;
const osThreadAttr_t sensUltrason_taskAttr = {
		.name = "sensUltrason_Task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Misc */
#define SOUND_SPEED_M_S		340

/*******************************************************************************
 * FUNCTIONS PROTOTYPES
 ******************************************************************************/

static void _hcsr04_GPIO_init(const hcsr04 *hcsr04_ptr);

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

static void _hcsr04_GPIO_init(const hcsr04 *hcsr04_ptr) {

	uint16_t trigger_pin = hcsr04_ptr->trigger_pin;
	GPIO_TypeDef * trigger_port = hcsr04_ptr->trigger_port;
	uint16_t echo_pin = hcsr04_ptr->echo_pin;
	GPIO_TypeDef * echo_port = hcsr04_ptr->echo_port;

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Trigger */
	HAL_GPIO_WritePin(trigger_port, trigger_pin, GPIO_PIN_RESET);
	GPIO_InitStruct.Pin = trigger_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(trigger_port, &GPIO_InitStruct);

	/* Echo */
	GPIO_InitStruct.Pin = echo_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(echo_port, &GPIO_InitStruct);

}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

/*****************************************
 * @brief Initialises GPIO pins for each hcsr04
 * @param None
 * @retval None
 ****************************************/
void hcsr04_GPIO_init(void) {
	for (unsigned int hcsr04_id=0; hcsr04_id<NB_HCSR04; hcsr04_id++) {
		const hcsr04 *hcsr04_ptr = hcsr04_ptr_array[hcsr04_id];
		_hcsr04_GPIO_init(hcsr04_ptr);
	}
}

/*****************************************
 * @brief Initialises TIM10 timer
 * @param None
 * @retval None
 ****************************************/
void hcsr04_TIM10_init(void) {
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = TIM10_PRESC;	// 1 us (for HCLK=96MHz)
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
}

/*****************************************
 * @brief Initialises semaphores and queues
 * @param None
 * @retval None
 ****************************************/
void hcsr04_semaph_queues_init(void) {
	hcsr04_binSemaph_run_handler = xSemaphoreCreateBinaryStatic( &hcsr04_binSemaph_run_buffer );
	assert_param(hcsr04_binSemaph_run_handler != NULL);
	xSemaphoreTake(hcsr04_binSemaph_run_handler,0);
	hcsr04_binSemaph_echo_handler = xSemaphoreCreateBinaryStatic( &hcsr04_binSemaph_echo_buffer );
	assert_param(hcsr04_binSemaph_echo_handler != NULL);
	xSemaphoreTake(hcsr04_binSemaph_echo_handler,0);

	for (unsigned int queue_index=0; queue_index<NB_HCSR04; queue_index++) {
		hcsr04_queue_handler[queue_index] = xQueueCreateStatic(HCSR04_QUEUE_LENGTH, sizeof(int),
				hcsr04_queue_stbuff[queue_index], &(hcsr04_queue_buffer[queue_index]));
	}
}


/*****************************************
 * @brief Task in charge of gathering data from each sensor.
 * It collects one data for each sensor each time its semaphore is opened.
 * @param argument = NULL (not used)
 * @retval None
 ****************************************/
void hcsr04_Task(void *argument) {
	while(1) {
		// 1) Wf being asked to run once
		while (xQueueSemaphoreTake(hcsr04_binSemaph_run_handler, portMAX_DELAY) == pdFALSE) {};
		// Per each hcsr04:
		for (unsigned int hcsr04_index=0; hcsr04_index<NB_HCSR04; hcsr04_index++) {
			// 2) Enable trigger (H, delay 20us, L)
			HAL_GPIO_WritePin(hcsr04_ptr_array[hcsr04_index]->trigger_port,
					hcsr04_ptr_array[hcsr04_index]->trigger_pin, GPIO_PIN_SET);
			systools_delay_us_nops(20);
			HAL_GPIO_WritePin(hcsr04_ptr_array[hcsr04_index]->trigger_port,
					hcsr04_ptr_array[hcsr04_index]->trigger_pin, GPIO_PIN_RESET);
			// 3) Wf echo rise
			HAL_TIM_Base_Stop_IT(&htim10);
			__HAL_TIM_SET_COUNTER(&htim10,0);
			int distance_mm = -1;
			if ( xQueueSemaphoreTake(hcsr04_binSemaph_echo_handler, pdMS_TO_TICKS(3)) == pdTRUE ) {
				// Ultrasonic sensor replied
				unsigned int count = __HAL_TIM_GET_COUNTER(&htim10);
				unsigned int count_us = count/TIM10_FREQ_MHZ;
				distance_mm = count_us*SOUND_SPEED_M_S/(2*1000);
			} else {
			}
			xQueueSendToBack(hcsr04_queue_handler[hcsr04_index], &distance_mm, 0);
		}
	}
}

/*****************************************
 * @brief
 * @param hcsr04_index: position of the specific hcsr04 that has interrupted.
 * This position refers to the position the sensor takes in hcsr04_ptr_array.
 * @retval None
 ****************************************/
void hcsr04_ISR(unsigned int hcsr04_index) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	GPIO_PinState pin_state = HAL_GPIO_ReadPin(
			hcsr04_ptr_array[hcsr04_index]->echo_port,
			hcsr04_ptr_array[hcsr04_index]->echo_pin);
	if (pin_state  == GPIO_PIN_SET) {		// Rising edge
		HAL_TIM_Base_Start_IT(&htim10);
	} else if (pin_state == GPIO_PIN_RESET) {	// Falling edge
		HAL_TIM_Base_Stop_IT(&htim10);
		xSemaphoreGiveFromISR(hcsr04_binSemaph_echo_handler, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
