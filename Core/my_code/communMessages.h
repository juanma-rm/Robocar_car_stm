/******************************************************************************
 * @file communMessages
 * @brief Definition of messages used in telemetry (which parameters are sent
 * by the car (outgoing), which ones are received from the control (incoming)
 * and the order in which they are received / sent
 ******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef COMMUNMESSAGES_H_
#define COMMUNMESSAGES_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * DEFINES
 ******************************************************************************/

/* For signed fields:
 * 	- INT16_MIN (-32768) will be considered as error
 * 	- Negative values refer to backward or left. Ex: -30 in linear speed refers to 30 backward
 * 	- Positive values refer to forward or right
 * For unsigned fields:
 *  - UINT16_MAX (+65535) will be considered as error
 */

/* Car incoming message structure:
 * 		Field 0: working mode. Unsigned. 0: manual, 1: automatic
 * 		Field 1: manual control / Y-axis. Signed: [-100, +100] %
 * 		Field 2: manual control / X-axis. Signed: [-100, +100] %
 * 		Field 3: automatic control / setpoint speed (mm/s) / Y-axis. Signed: [-32767, +32767] mm/s
 * 		Field 4: automatic control / setpoint speed (??) / X-axis. Signed: [??]
 */
#define MESSAGE_IN_NB_FIELDS				5
#define MESSAGE_IN_POS_WORKMODE				0
#define MESSAGE_IN_POS_MANCTRLY_PERC		1
#define MESSAGE_IN_POS_MANCTRLX_PERC		2
#define MESSAGE_IN_POS_AUTCTRL_SPEEDY_MMS	3
#define MESSAGE_IN_POS_AUTCTRL_SPEEDX_MMS	4

/* Car outgoing message structure:
 * 		Field 0: working mode. Unsigned. 0: manual, 1: automatic
 * 		Field 1: manual control / Y-axis. Signed: [-100, +100] %
 * 		Field 2: manual control / X-axis. Signed: [-100, +100] %
 * 		Field 3: automatic control / setpoint speed (mm/s) / Y-axis. Signed: [-32767, +32767] mm/s
 * 		Field 4: automatic control / setpoint speed (??) / X-axis. Signed: [??]
 * 		Field 5: linear speed (mm/s). Unsigned: [-32767, +32767] mm/s
 * 		Field 6: left wheel speed (rpm). Signed: [-32767, +32767] rpm
 * 		Field 7: right wheel speed (rpm). Signed: [-32767, +32767] rpm
 *		Field 8: left distance (mm). Unsigned. [0,65535] mm
 *		Field 9: right distance (mm). Unsigned. [0,65535] mm
 *
 *		Errors, other...
 */
#define MESSAGE_OUT_NB_FIELDS				10
#define MESSAGE_OUT_POS_WORKMODE			0
#define MESSAGE_OUT_POS_MANCTRLY_PERC		1
#define MESSAGE_OUT_POS_MANCTRLX_PERC		2
#define MESSAGE_OUT_POS_AUTCTRL_SPEEDY_MMS	3
#define MESSAGE_OUT_POS_AUTCTRL_SPEEDX_MMS	4
#define MESSAGE_OUT_POS_LINSPEED_MMS		5
#define MESSAGE_OUT_POS_LSPEED_RPM			6
#define MESSAGE_OUT_POS_RSPEED_RPM			7
#define MESSAGE_OUT_POS_LDIST_MM			8
#define MESSAGE_OUT_POS_RDIST_MM			9

typedef struct {
	uint16_t workmode;
	bool workmode_err;
	int16_t manctrly_perc;
	bool manctrly_err;
	int16_t manctrlx_perc;
	bool manctrlx_err;
	int16_t autctrl_speedy_mms;
	bool autctrl_speedy_err;
	int16_t autctrl_speedx_mms;
	bool autctrl_speedx_err;
} Message_struct_in;

typedef struct {
	uint16_t workmode;
	bool workmode_err;
	int16_t manctrly_perc;
	bool manctrly_err;
	int16_t manctrlx_perc;
	bool manctrlx_err;
	int16_t autctrl_speedy_mms;
	bool autctrl_speedy_err;
	int16_t autctrl_speedx_mms;
	bool autctrl_speedx_err;
	int16_t linspeed_mms;
	bool linspeed_err;
	int16_t lspeed_rpm;
	bool lspeed_err;
	int16_t rspeed_rpm;
	bool rspeed_err;
	uint16_t ldist_mm;
	bool ldist_err;
	uint16_t rdist_mm;
	bool rdist_err;
} Message_struct_out;

/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* COMMUNMESSAGES_H_ */
