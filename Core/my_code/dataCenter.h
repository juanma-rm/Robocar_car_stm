/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file dataCenter.h
 * @version v1.0
 * @brief Header for dataCenter.c file
******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef DATACENTER_H_
#define DATACENTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include <stdbool.h>

/*******************************************************************************
 * USER
 ******************************************************************************/

/*******************************************************************************
 * DATA STRUCTS / ENUMS
 ******************************************************************************/

// Stores current car status coming from robot_ctrl
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
} Robot_ctrl_struct;

// Stores current order coming from telemetry to robot_ctrl
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
} Tlmt_cmd_struct;

/*******************************************************************************
 * DEFINES
 ******************************************************************************/



/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/



/*******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************/

void dataCenter_task(void *argument);


/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DATACENTER_H_ */
