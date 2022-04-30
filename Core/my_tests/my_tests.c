/******************************************************************************
 * @file my_tests.c
 * @brief
 *
 * \todo complete description
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "systools.h"

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/



/*******************************************************************************
 * FUNCTIONS DEFINITIONS
 ******************************************************************************/

/*****************************************
 * @brief test if diff_time function works as expected
 * @retval None
 ****************************************/
void test_ts_diff_time() {
	static int t1 = 0;
	static int t2 = 0;
	static int tmax = 50;
	static int tincr = 7;

	t2 += tincr;
	t2 %= tmax;
	int int_diff_time = systools_diff_time(t1,t2,tmax);

	char str_out_time[40];
	sprintf(str_out_time, "Difference in time: %d,%d,%d\n\r", t1,t2,int_diff_time);
	systools_transm_usart3((uint8_t *)str_out_time, strlen(str_out_time), 10);
	t1 = t2;
}

void test_systools_delay_us_nops(void) {
	extern TIM_HandleTypeDef htim10;
	HAL_TIM_Base_Stop_IT(&htim10);
	__HAL_TIM_SET_COUNTER(&htim10,0);
	HAL_TIM_Base_Start_IT(&htim10);
	systools_delay_us_nops(100);
	HAL_TIM_Base_Stop_IT(&htim10);
	unsigned int tim10_cycles_us = __HAL_TIM_GET_COUNTER(&htim10);
	char str_out[40];
	sprintf(str_out, "tim10_cycles_us: %d\n\r", tim10_cycles_us);
	systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
}
/*
void test_PWM(void) {
	while(1) {
		for (unsigned int i=0; i<=100; i++) {
			_set_PWM(&pwm_L, i);
			_set_PWM(&pwm_R, i);
			systools_delay_us_nops(100000);
		}
	}


	while(1) {
		systools_delay_us_nops(10000);
	}
}

#include <stdio.h>
void test_encoder(void) {
	  while (1)
	  {
		  uint8_t message[50] = {'\0'};
		  int speed_mm_s = (int)(read_speed_mm_s(&encoder_L));
		  sprintf(message, "Encoder mm/s: %d\n\r", speed_mm_s);
		  systools_transm_usart3(message, sizeof(message), 100);
		  systools_delay_us_nops(TIME_BET_MEAS_MS*1000);
	  }
}

void test_dir(void) {
	while(1) {
		HAL_GPIO_WritePin(dir_L.forw_gpio_port, dir_L.forw_gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_L.back_gpio_port, dir_L.back_gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_R.forw_gpio_port, dir_R.forw_gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dir_R.back_gpio_port, dir_R.back_gpio_pin, GPIO_PIN_SET);
		systools_delay_us_nops(100000);
		HAL_GPIO_WritePin(dir_L.forw_gpio_port, dir_L.forw_gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_L.back_gpio_port, dir_L.back_gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_R.forw_gpio_port, dir_R.forw_gpio_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dir_R.back_gpio_port, dir_R.back_gpio_pin, GPIO_PIN_RESET);
		systools_delay_us_nops(100000);
	}
}

#include <stdio.h>
void test_motor(void) {
	while(1) {
		float speed_mm_s;
		uint8_t message[50] = {'\0'};
		// Forward
		for (unsigned int pwm_perc=0; pwm_perc<=100; pwm_perc+=10) {
			motor_set(MOTOR_L, MOTOR_FORWARD, pwm_perc);
			systools_delay_us_nops(500000);
			speed_mm_s = read_speed_mm_s(MOTOR_L);
			sprintf(message, "Encoder mm/s: %d\n\r", (int)(speed_mm_s*100));
			systools_transm_usart3(message, sizeof(message), 100);
		}
		motor_set(MOTOR_L, MOTOR_FREE, 0);
		systools_delay_us_nops(1000000);
		// Backward
		for (unsigned int pwm_perc=0; pwm_perc<=100; pwm_perc+=10) {
			motor_set(MOTOR_L, MOTOR_BACKWARD, pwm_perc);
			systools_delay_us_nops(500000);
			speed_mm_s = read_speed_mm_s(MOTOR_L);
			sprintf(message, "Encoder mm/s: %d\n\r", (int)(speed_mm_s*100));
			systools_transm_usart3(message, sizeof(message), 100);
		}
		motor_set(MOTOR_L, MOTOR_FREE, 0);
		systools_delay_us_nops(1000000);
		// Brake
		motor_set(MOTOR_L, MOTOR_FORWARD, 70);	// Previous mid speed before brake
		systools_delay_us_nops(3000000);
		motor_set(MOTOR_L, MOTOR_BRAKE, 0);
		systools_delay_us_nops(3000000);
		// Free
		motor_set(MOTOR_L, MOTOR_FORWARD, 70);	// Previous mid speed before free
		systools_delay_us_nops(3000000);
		motor_set(MOTOR_L, MOTOR_FREE, 0);
		systools_delay_us_nops(3000000);

	}
}
*/
