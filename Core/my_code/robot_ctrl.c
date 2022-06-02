/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file robot_ctrl.c
 * @version v1.0
 * @brief File containing functions related to the robot movement
 *
 * @todo
 *
 * Requirements:
 * 	1)
 *
 * Warnings!
 * 	1)
 *
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

// Std
#include <string.h>
// Freertos and hardware
#include "cmsis_os.h"
#include "semphr.h"
#include "queue.h"
// Project
#include "robot_ctrl.h"
#include "motor_driver.h"
#include "systools.h"
#include "dataCenter.h"

/*******************************************************************************
 * DATA STRUCTS
 ******************************************************************************/



/*******************************************************************************
 * USER
 ******************************************************************************/




/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

#define DISCRETE_PERIOD_MS	100

/* Timer */

/* Semaphores and queues definitions */

// Queue for data input
#define TLMT_CMD_QUEUE_LENGTH	1
xQueueHandle tlmt_cmd_queue_handler;
static StaticQueue_t tlmt_cmd_queue_buffer;
static uint8_t tlmt_cmd_queue_stbuff[TLMT_CMD_QUEUE_LENGTH*sizeof(Tlmt_cmd_struct)/sizeof(uint8_t)];
				// the queue will store Tlmt_cmd_struct elements

// Queue for data output
#define FROM_ROBOT_CTRL_QUEUE_LENGTH	1
xQueueHandle from_robot_ctrl_queue_handler;
static StaticQueue_t from_robot_ctrl_queue_buffer;
static uint8_t from_robot_ctrl_queue_stbuff[FROM_ROBOT_CTRL_QUEUE_LENGTH*sizeof(Robot_ctrl_struct)/sizeof(uint8_t)];
				// the queue will store Robot_ctrl_struct elements

/* Task handle / attributes */
osThreadId_t robot_ctrl_taskHandle;
const osThreadAttr_t robot_ctrl_taskAttr = {
		.name = "robot_ctrl_task",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

/*****************************************
 * @brief Initialise semaphores and queues
 ****************************************/
static void _semaph_queues_init(void) {
	tlmt_cmd_queue_handler = xQueueCreateStatic(TLMT_CMD_QUEUE_LENGTH, sizeof(Tlmt_cmd_struct),
			tlmt_cmd_queue_stbuff, &(tlmt_cmd_queue_buffer));
	from_robot_ctrl_queue_handler = xQueueCreateStatic(FROM_ROBOT_CTRL_QUEUE_LENGTH, sizeof(Robot_ctrl_struct),
			from_robot_ctrl_queue_stbuff, &(from_robot_ctrl_queue_buffer));
}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

/*****************************************
 * @brief
 * @param
 ****************************************/
void robot_ctrl_init(void) {
	motor_ctrl_init();
	_semaph_queues_init();
}

/*****************************************
 * @brief
 * @param
 ****************************************/
void robot_ctrl_task(void *argument) {

	Tlmt_cmd_struct tlmt_cmd;
	memset(&tlmt_cmd, 0, sizeof(Tlmt_cmd_struct));
	Robot_ctrl_struct curr_state;
	memset(&curr_state, 0, sizeof(Tlmt_cmd_struct)); // By default, workmode = 0 (stop)

	while(1) {

		// 1) If new command available, update control parameters; else, use parameters from previous iteration
		if (xQueueReceive(tlmt_cmd_queue_handler,  &tlmt_cmd, 0) == pdTRUE) {
			if (tlmt_cmd.workmode_updated == true) {
				curr_state.workmode = tlmt_cmd.workmode;
				curr_state.workmode_updated = true;
			} else {
				curr_state.workmode_updated = false;
			}
			if (tlmt_cmd.autctrl_speedx_updated == true) {
				curr_state.autctrl_speedx_mms = tlmt_cmd.autctrl_speedx_mms;
				curr_state.autctrl_speedx_updated = true;
			} else {
				curr_state.autctrl_speedx_updated = false;
			}
			if (tlmt_cmd.autctrl_speedy_updated == true) {
				curr_state.autctrl_speedy_mms = tlmt_cmd.autctrl_speedy_mms;
				curr_state.autctrl_speedy_updated = true;
			} else {
				curr_state.autctrl_speedy_updated = false;
			}
			if (tlmt_cmd.manctrlx_updated == true) {
				curr_state.manctrlx_perc = tlmt_cmd.manctrlx_perc;
				curr_state.manctrlx_updated = true;
			} else {
				curr_state.manctrlx_updated = false;
			}
			if (tlmt_cmd.manctrly_updated == true) {
				curr_state.manctrly_perc = tlmt_cmd.manctrly_perc;
				curr_state.manctrly_updated = true;
			} else {
				curr_state.manctrly_updated = false;
			}
		}

		// 2) Read current speed

		float speed_L_mm_s;
		read_speed(MOTOR_L, DISCRETE_PERIOD_MS, &speed_L_mm_s, &(curr_state.lspeed_rpm));
		curr_state.lspeed_updated = true;

		float speed_R_mm_s;
		read_speed(MOTOR_R, DISCRETE_PERIOD_MS, &speed_R_mm_s, &(curr_state.rspeed_rpm));
		curr_state.rspeed_updated = true;

		curr_state.linspeed_mms = ((int)speed_L_mm_s + (int)speed_R_mm_s)/2;
		curr_state.linspeed_updated = true;

		// 3) Perform calculations according to current workmode
		unsigned int motor_L_dir;
		unsigned int motor_L_pwm;
		unsigned int motor_R_dir;
		unsigned int motor_R_pwm;

		// 3.1) Workmode = 0: stop
		if (curr_state.workmode == 0) {
			motor_L_dir = MOTOR_FORWARD;
			motor_L_pwm = 0;
			motor_R_dir = MOTOR_FORWARD;
			motor_R_pwm = 0;

		// 3.2) Workmode = 1: manual control
		} else if (curr_state.workmode == 1) {
			unsigned int manctrly_perc_abs;
			// Direction OY: forward / backward
			if (curr_state.manctrly_perc < 0) {
				motor_L_dir = MOTOR_BACKWARD;
				motor_R_dir = MOTOR_BACKWARD;
				manctrly_perc_abs = -curr_state.manctrly_perc;
			} else {
				motor_L_dir = MOTOR_FORWARD;
				motor_R_dir = MOTOR_FORWARD;
				manctrly_perc_abs = curr_state.manctrly_perc;
			}
			// Direction OX and speed: left if negative, right if positive
			// Proportionally: -100% means turning left on site; +100% means turning right on site;
			// 0% means going straight;
			// -50% means turning left reducing right motor to 50% of OY speed
			// 70% means turning right reducing left motor to 70% of OY speed
			if (curr_state.manctrlx_perc == 0) {
				motor_L_pwm = manctrly_perc_abs;
				motor_R_pwm = manctrly_perc_abs;
			} else if (curr_state.manctrlx_perc < 0) {
				motor_L_pwm = manctrly_perc_abs*(100+1*curr_state.manctrlx_perc)/100;
				motor_R_pwm = manctrly_perc_abs;
			} else if (curr_state.manctrlx_perc > 0) {
				motor_L_pwm = manctrly_perc_abs;
				motor_R_pwm = manctrly_perc_abs*(100-curr_state.manctrlx_perc)/100;
			}

		// 3.3) Workmode = 2: automatic control (PID)
		} else if (curr_state.workmode == 2) {
			// Get PID output
//			motor_L_dir = ;
//			motor_L_pwm = ;
//			motor_R_dir = ;
//			motor_R_pwm = ;

		// 3.4) Other (this branch should not be taken): stop
		} else {
			motor_L_dir = MOTOR_FORWARD;
			motor_L_pwm = 0;
			motor_R_dir = MOTOR_FORWARD;
			motor_R_pwm = 0;
		}

		// 4) Perform control
		motor_set(MOTOR_L, motor_L_dir, motor_L_pwm);
		motor_set(MOTOR_R, motor_R_dir, motor_R_pwm);

		// 5) Send current state to data center
		// Remove last state in case it has not been processed yet
		Robot_ctrl_struct unused;
		xQueueReceive(from_robot_ctrl_queue_handler,  &unused, 0);
		xQueueSend(from_robot_ctrl_queue_handler, &curr_state, 0);

		// 6) Wait
		vTaskDelay(pdMS_TO_TICKS(DISCRETE_PERIOD_MS));
	}
}


