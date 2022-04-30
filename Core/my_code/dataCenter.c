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

//#define PRINT_STATUS		// Uncomment for printing status (serial output)
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
	uint16_t workmode;
	bool workmode_updated;
	int16_t manctrly_perc;
	bool manctrly_updated;
	int16_t manctrlx_perc;
	bool manctrlx_updated;
	int16_t autctrl_speedy_mms;
	bool autctrl_speedy_updated;
	int16_t autctrl_speedx_mms;
	bool autctrl_speedx_updated;
	int16_t linspeed_mms;
	bool linspeed_updated;
	int16_t lspeed_rpm;
	bool lspeed_updated;
	int16_t rspeed_rpm;
	bool rspeed_updated;
	int16_t ldist_mm;
	bool ldist_updated;
	int16_t rdist_mm;
	bool rdist_updated;
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

/* robot_ctrl */
extern xQueueHandle tlmt_cmd_queue_handler;
extern xQueueHandle from_robot_ctrl_queue_handler;



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
			"Wheel speed (rpm)",
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
	Tlmt_cmd_struct tlmt_cmd;
	memset(&tlmt_cmd, 0, sizeof(Tlmt_cmd_struct));

	// Print variables
	bool first_print = true;
	unsigned int last_print = 0;

	// Task flow
	while(1) {

		// Receive from sensors (speed from motor_ctrl encoders)
		Robot_ctrl_struct data_from_robot_ctrl;
		if ( xQueueReceive(from_robot_ctrl_queue_handler,  &data_from_robot_ctrl, 0) == pdTRUE ) {
			car_state.autctrl_speedx_mms = data_from_robot_ctrl.autctrl_speedx_mms;
			car_state.autctrl_speedy_mms = data_from_robot_ctrl.autctrl_speedy_mms;
			car_state.linspeed_mms = data_from_robot_ctrl.linspeed_mms;
			car_state.lspeed_rpm = data_from_robot_ctrl.lspeed_rpm;
			car_state.rspeed_rpm = data_from_robot_ctrl.rspeed_rpm;
			car_state.manctrlx_perc = data_from_robot_ctrl.manctrlx_perc;
			car_state.manctrly_perc = data_from_robot_ctrl.manctrly_perc;
			car_state.workmode = data_from_robot_ctrl.workmode;

			car_state.autctrl_speedx_updated = data_from_robot_ctrl.autctrl_speedx_updated;
			car_state.autctrl_speedy_updated = data_from_robot_ctrl.autctrl_speedy_updated;
			car_state.linspeed_updated = data_from_robot_ctrl.linspeed_updated;
			car_state.lspeed_updated = data_from_robot_ctrl.lspeed_updated;
			car_state.rspeed_updated = data_from_robot_ctrl.rspeed_updated;
			car_state.manctrlx_updated = data_from_robot_ctrl.manctrlx_updated;
			car_state.manctrly_updated = data_from_robot_ctrl.manctrly_updated;
			car_state.workmode_updated = data_from_robot_ctrl.workmode_updated;
		} else {
			car_state.autctrl_speedx_updated = false;
			car_state.autctrl_speedy_updated = false;
			car_state.linspeed_updated = false;
			car_state.lspeed_updated = false;
			car_state.rspeed_updated = false;
			car_state.manctrlx_updated = false;
			car_state.manctrly_updated = false;
			car_state.rspeed_updated = false;
			car_state.workmode_updated = false;
		}

		// Receive from sensors (distance from hcsr04)
		// Receive data from hcsr04_L sensor
		if ( xQueueReceive(hcsr04_queue_handler[HCSR04_L_ID],  &(car_state.ldist_mm), 0) == pdTRUE ) {
			car_state.ldist_updated = true;
		} else {
			car_state.ldist_updated = false;
		}
		// Receive data from hcsr04_R sensor
		if ( xQueueReceive(hcsr04_queue_handler[HCSR04_R_ID],  &(car_state.rdist_mm), 0) == pdTRUE ) {
			car_state.rdist_updated = true;
		} else {
			car_state.rdist_updated = false;
		}

		// Receive from spi connection
		Message_struct_in message_in;
		if ( xQueueReceive(communespspi_in_queue_handler,  &message_in, 0) == pdTRUE ) {
			if (!message_in.workmode_err) {
				tlmt_cmd.workmode = message_in.workmode;
				tlmt_cmd.workmode_updated = true;
			} else {
				tlmt_cmd.workmode_updated = false;
			}
			if (!message_in.manctrly_err) {
				tlmt_cmd.manctrly_perc = message_in.manctrly_perc;
				tlmt_cmd.manctrly_updated = true;
			} else {
				tlmt_cmd.manctrly_updated = false;
			}
			if (!message_in.manctrlx_err) {
				tlmt_cmd.manctrlx_perc = message_in.manctrlx_perc;
				tlmt_cmd.manctrlx_updated = true;
			} else {
				tlmt_cmd.manctrlx_updated = false;
			}
			if (!message_in.autctrl_speedy_err) {
				tlmt_cmd.autctrl_speedy_mms = message_in.autctrl_speedy_mms;
				tlmt_cmd.autctrl_speedy_updated = true;
			} else {
				tlmt_cmd.autctrl_speedy_updated = false;
			}
			if (!message_in.autctrl_speedx_err) {
				tlmt_cmd.autctrl_speedx_mms = message_in.autctrl_speedx_mms;
				tlmt_cmd.autctrl_speedx_updated = true;
			} else {
				tlmt_cmd.autctrl_speedx_updated = false;
			}
		}
		// Temp manual commands to robot_ctrl
//		tlmt_cmd.workmode = 0;
//		tlmt_cmd.workmode = 1;
//		tlmt_cmd.workmode_updated = true;
//		tlmt_cmd.manctrlx_perc = 0;
//		tlmt_cmd.manctrlx_updated = true;
//		tlmt_cmd.manctrly_perc = 30;
//		tlmt_cmd.manctrly_updated = true;

		// Send new command to robot_ctrl
		// Remove last command in case it has not been processed yet
		Tlmt_cmd_struct unused;
		xQueueReceive(tlmt_cmd_queue_handler,  &unused, 0);
		xQueueSend(tlmt_cmd_queue_handler, &tlmt_cmd, 0);

		// Send new data to be sent via spi connection
		if (uxQueueMessagesWaiting(communespspi_out_queue_handler) == 0) {
			Message_struct_out message_out;
			message_out.workmode_err = false;
			message_out.workmode = car_state.workmode;
			message_out.manctrly_err = false;
			message_out.manctrly_perc = car_state.manctrly_perc;
			message_out.manctrlx_err = false;
			message_out.manctrlx_perc = car_state.manctrlx_perc;
			message_out.autctrl_speedy_err = false;
			message_out.autctrl_speedy_mms = car_state.autctrl_speedy_mms;
			message_out.autctrl_speedx_err = false;
			message_out.autctrl_speedx_mms = car_state.autctrl_speedx_mms;
			message_out.linspeed_err = false;
			message_out.linspeed_mms = car_state.linspeed_mms;
			message_out.lspeed_err = false;
			message_out.lspeed_rpm = car_state.lspeed_rpm;
			message_out.rspeed_err = false;
			message_out.rspeed_rpm = car_state.rspeed_rpm;
			message_out.ldist_err = false;
			message_out.ldist_mm = car_state.ldist_mm;
			message_out.rdist_err = false;
			message_out.rdist_mm = car_state.rdist_mm;
			xQueueSend(communespspi_out_queue_handler, &message_out, 0);
		}

		// Print via uart
#if defined(PRINT_STATUS)
		if (first_print || systools_diff_time(last_print,osKernelGetTickCount(),UINT16_MAX)>TICKS_BETW_PRINT ) {
			systools_print_status(
					(int16_t) car_state.workmode, car_state.manctrly_perc, car_state.manctrlx_perc,
					car_state.autctrl_speedy_mms, car_state.autctrl_speedx_mms,
					car_state.linspeed_mms, car_state.lspeed_rpm, car_state.rspeed_rpm,
					car_state.ldist_mm, car_state.rdist_mm);
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
