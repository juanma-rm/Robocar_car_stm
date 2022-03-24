/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file dataCenter.c
 * @version v1.0
 * @brief @todo
 * 	Requirements: @todo

 * Warning!! (any warning?) @todo
******************************************************************************/

#define PRINT_STATUS		// Uncomment for printing status (serial output)
#define TICKS_BETW_PRINT	(500*configTICK_RATE_HZ/1000)

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

// Std
#include <string.h>
#include <stdio.h>
#include <limits.h>
// Freertos and hardware
#include "cmsis_os.h"
#include "semphr.h"
#include "queue.h"
// Project
#include "dataCenter.h"
#include "hcsr04.h"
#include "communEspSpi.h"
#include "systools.h"

/*******************************************************************************
 * DATA STRUCTS / ENUMS
 ******************************************************************************/

typedef struct {
	uint16_t st_workmode;
	bool st_workmode_updated;
	int16_t st_manctrly_perc;
	bool st_manctrly_updated;
	int16_t st_manctrlx_perc;
	bool st_manctrlx_updated;
	int16_t st_autctrl_speedy_mms;
	bool st_autctrl_speedy_updated;
	int16_t st_autctrl_speedx_mms;
	bool st_autctrl_speedx_updated;
	int16_t st_linspeed_mms;
	bool st_linspeed_updated;
	int16_t st_lspeed_rpm;
	bool st_lspeed_updated;
	int16_t st_rspeed_rpm;
	bool st_rspeed_updated;
	int16_t st_ldist_mm;
	bool st_ldist_updated;
	int16_t st_rdist_mm;
	bool st_rdist_updated;
	uint16_t tlmt_workmode;
	bool tlmt_workmode_updated;
	int16_t tlmt_manctrly_perc;
	bool tlmt_manctrly_updated;
	int16_t tlmt_manctrlx_perc;
	bool tlmt_manctrlx_updated;
	int16_t tlmt_autctrl_speedy_mms;
	bool tlmt_autctrl_speedy_updated;
	int16_t tlmt_autctrl_speedx_mms;
	bool tlmt_autctrl_speedx_updated;
} Car_state_struct;

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Task handle / attributes */
osThreadId_t dataCenter_taskHandle;
const osThreadAttr_t dataCenter_taskAttr = {
		.name = "dataCenter_task",
		.stack_size = 128 * 16,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Semaphores and queues definitions */

/* hcsr04 */
extern xSemaphoreHandle hcsr04_binSemaph_run_handler;
extern xQueueHandle hcsr04_queue_handler[NB_HCSR04];

/* communEspSpi */
extern xQueueHandle communespspi_in_queue_handler;
extern xQueueHandle communespspi_out_queue_handler;


/*******************************************************************************
 * PRIVATE VARIABLES
 ******************************************************************************/



/*******************************************************************************
 * FUNCTIONS PROTOTYPES: PRIVATE
 ******************************************************************************/

static void systools_print_status(
		int workmode, int manctrly_per, int manctrlx_per,
		int autctrl_speedy_mms, int autctrl_speedx_mms, int linspeed_mms,
		int lspeed_rpm, int rspeed_rpm, int ldist_mm, int rdist_mm );

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

/*****************************************
 * @brief prints current car status via usart3
 ****************************************/
static void systools_print_status(
		int workmode, int manctrly_per, int manctrlx_per,
		int autctrl_speedy_mms, int autctrl_speedx_mms, int linspeed_mms,
		int lspeed_rpm, int rspeed_rpm, int ldist_mm, int rdist_mm ) {


	#define NB_LINES	6
	#define COL_NB		3
	#define COL0_LENGTH	32	// Nb of chars
	#define COL1_LENGTH	12
	#define COL2_LENGTH	12
	#define COL_LENGTH	(COL0_LENGTH + COL1_LENGTH + COL2_LENGTH)

	const char col0_mess[NB_LINES][COL0_LENGTH] = {
			"Workmode",
			"Manual setpoint (%)",
			"Automatic setpoint (mms)",
			"Linear speed (mm/s)",
			"Wheel speed (mm/s)",
			"Distance to obstacle (mm)"
	};
	char col1_mess[NB_LINES][COL1_LENGTH] = {0};
	sprintf(col1_mess[0], "%d", workmode);
	sprintf(col1_mess[1], "%d", manctrly_per);
	sprintf(col1_mess[2], "%d", autctrl_speedy_mms);
	sprintf(col1_mess[3], "%d", linspeed_mms);
	sprintf(col1_mess[4], "%d", lspeed_rpm);
	sprintf(col1_mess[5], "%d", ldist_mm);
	char col2_mess[NB_LINES][COL2_LENGTH] = {0};
	snprintf(col2_mess[0], COL2_LENGTH, "-");
	snprintf(col2_mess[1], COL2_LENGTH,"%d", manctrlx_per);
	snprintf(col2_mess[2], COL2_LENGTH,"%d", autctrl_speedx_mms);
	snprintf(col2_mess[3], COL2_LENGTH,"-");
	snprintf(col2_mess[4], COL2_LENGTH,"%d", rspeed_rpm);
	snprintf(col2_mess[5], COL2_LENGTH,"%d", rdist_mm);

	/* Clear console */
	char line_clear[] = "\e[1;1H\e[2J";
	systools_transm_usart3((uint8_t *)line_clear, strlen(line_clear), 10);
	/* Header message */
	char line_header[] = "SYSTEM STATUS:\n\n\r";
	systools_transm_usart3((uint8_t *)line_header, strlen(line_header), 10);
	/* Values */
	char col0[COL0_LENGTH] = "";
	char col1[COL1_LENGTH] = "";
	char col2[COL2_LENGTH] = "";
	for (unsigned int line=0; line<NB_LINES; line++) {
		sprintf(col0, "%s", col0_mess[line]);
		for (unsigned int pos=strlen(col0); pos<COL0_LENGTH; pos++) {
			col0[pos] = ' ';
		}
		systools_transm_usart3((uint8_t *)col0, COL0_LENGTH, 10);
		sprintf(col1, "%s", col1_mess[line]);
		for (unsigned int pos=strlen(col1); pos<COL1_LENGTH; pos++) {
			col1[pos] = ' ';
		}
		systools_transm_usart3((uint8_t *)col1, COL1_LENGTH, 10);
		sprintf(col2, "%s", col2_mess[line]);
		for (unsigned int pos=strlen(col2); pos<COL2_LENGTH; pos++) {
			col2[pos] = ' ';
		}
		systools_transm_usart3((uint8_t *)col2, COL2_LENGTH, 10);
		systools_transm_usart3((uint8_t *)"\n\r", 4, 10);
	}

}


/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

/*****************************************
 * @brief Task in charge of interconnecting data (queues) between rest of tasks
 * @param argument = NULL (not used)
 ****************************************/
void dataCenter_task(void *argument) {
	// Structure to allocate data
	Car_state_struct car_state;
	memset(&car_state, 0, sizeof(Car_state_struct));

	// Print variables
	bool first_print = true;
	unsigned int last_print = 0;

	// Task flow
	while(1) {

		// Receive data from encoder_L sensor
		//@todo
		// Receive data from encoder_R sensor
		//@todo
		// Receive data for linear speed
		//@todo
		// Receive data from hcsr04_L sensor
		if ( xQueueReceive(hcsr04_queue_handler[HCSR04_L_ID],  &(car_state.st_ldist_mm), 0) == pdTRUE ) {
			car_state.st_ldist_updated = true;
		} else {
			car_state.st_ldist_updated = false;
		}
		// Receive data from hcsr04_R sensor
		if ( xQueueReceive(hcsr04_queue_handler[HCSR04_R_ID],  &(car_state.st_rdist_mm), 0) == pdTRUE ) {
			car_state.st_rdist_updated = true;
		} else {
			car_state.st_rdist_updated = false;
		}

		// Receive from spi connection
		car_state.tlmt_workmode_updated = false;
		car_state.tlmt_manctrly_updated = false;
		car_state.tlmt_manctrlx_updated = false;
		car_state.tlmt_autctrl_speedy_updated = false;
		car_state.tlmt_autctrl_speedx_updated = false;
		Message_struct_in message_in;
		if ( xQueueReceive(communespspi_in_queue_handler,  &message_in, 0) == pdTRUE ) {
			if (!message_in.workmode_err) {
				car_state.tlmt_workmode = message_in.workmode;
				car_state.tlmt_workmode_updated = true;
			}
			if (!message_in.manctrly_err) {
				car_state.tlmt_manctrly_perc = message_in.manctrly_perc;
				car_state.tlmt_manctrly_updated = true;
			}
			if (!message_in.manctrlx_err) {
				car_state.tlmt_manctrlx_perc = message_in.manctrlx_perc;
				car_state.tlmt_manctrlx_updated = true;
			}
			if (!message_in.autctrl_speedy_err) {
				car_state.tlmt_autctrl_speedy_mms = message_in.autctrl_speedy_mms;
				car_state.tlmt_autctrl_speedy_updated = true;
			}
			if (!message_in.autctrl_speedx_err) {
				car_state.tlmt_autctrl_speedx_mms = message_in.autctrl_speedx_mms;
				car_state.tlmt_autctrl_speedx_updated = true;
			}
		}

		// Update st_workmode
		//@todo
		// Update st_manctrly_perc
		//@todo
		// Update st_manctrlx_perc
		//@todo
		// Update st_autctrl_speedy_mms
		//@todo
		// Update st_autctrl_speedx_mms
		//@todo

		// Send new data to be sent via spi connection
		if (uxQueueMessagesWaiting(communespspi_out_queue_handler) == 0) {
			Message_struct_out message_out;
			message_out.workmode_err = false;
			message_out.workmode = 0;
			message_out.manctrly_err = false;
			message_out.manctrly_perc = 80;
			message_out.manctrlx_err = false;
			message_out.manctrlx_perc = 0;
			message_out.autctrl_speedy_err = false;
			message_out.autctrl_speedy_mms = 150;
			message_out.autctrl_speedx_err = false;
			message_out.autctrl_speedx_mms = 150;
			message_out.linspeed_err = false;
			message_out.linspeed_mms = 150;
			message_out.lspeed_err = false;
			message_out.lspeed_rpm = 15;
			message_out.rspeed_err = false;
			message_out.rspeed_rpm = 15;
			message_out.ldist_err = false;
			message_out.ldist_mm = car_state.st_ldist_mm;
			message_out.rdist_err = false;
			message_out.rdist_mm = 150;
			xQueueSend(communespspi_out_queue_handler, &message_out, 0);
		}

		// Print via uart
#if defined(PRINT_STATUS)
		if (first_print || systools_diff_time(last_print,osKernelGetTickCount(),UINT16_MAX)>TICKS_BETW_PRINT ) {
			systools_print_status(
					(int16_t) car_state.st_workmode, car_state.st_manctrly_perc, car_state.st_manctrlx_perc,
					car_state.st_autctrl_speedy_mms, car_state.st_autctrl_speedx_mms,
					car_state.st_linspeed_mms, car_state.st_lspeed_rpm, car_state.st_rspeed_rpm,
					car_state.st_ldist_mm, car_state.st_rdist_mm);
			last_print = osKernelGetTickCount();
			first_print = false;
		}
#endif

		// Wait between iterations
		osDelay(25*configTICK_RATE_HZ/1000);
	}
}


/*******************************************************************************
 * FUNCTIONS DEFINITIONS: HAL / ISR
 ******************************************************************************/
