/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file motor_ctrl.h
 * @version v1.0
 * @brief Header for motor_ctrl.h.c file
******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/



/*******************************************************************************
 * USER
 ******************************************************************************/

// DIRECTION
#define MOTOR_FORWARD	0
#define MOTOR_BACKWARD	1
#define MOTOR_FREE		2
// MOTOR ID
#define MOTOR_L		0
#define MOTOR_R		1

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

void motor_ctrl_init(void);
void motor_set(unsigned int side, unsigned int dir, unsigned int pwm_perc);
void read_speed(unsigned int side, unsigned int time_bet_meas_ms, float *speed_mm_s, int16_t *speed_rpm);

/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H_ */
