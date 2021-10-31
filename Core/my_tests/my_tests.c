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


