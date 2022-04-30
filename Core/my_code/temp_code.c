/*

  void StartDefaultTask(void *argument) {
	char str_out[40] = "Hello from StartDefaultTask!\n\r";
	systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);

	while(1) {

		// Receive data from hcsr04 sensors
		int hcsr04_data[NB_HCSR04];
		for (unsigned int hcsr04_index=0; hcsr04_index<NB_HCSR04; hcsr04_index++) {
			int *hcsr04_data_ptr = &(hcsr04_data[hcsr04_index]);
			if ( xQueueReceive(hcsr04_queue_handler[hcsr04_index],  hcsr04_data_ptr, 0) == pdTRUE ) {
				if (*hcsr04_data_ptr != -1) {
					sprintf(str_out, "hcsr04 %d: %d\n\r", hcsr04_index, *hcsr04_data_ptr);
					//systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
				} else {
					sprintf(str_out, "hcsr04 %d: sensor not available\n\r", hcsr04_index);
				}
			} else {
				sprintf(str_out, "hcsr04 %d: no new data received\n\r", hcsr04_index);
			}
//			systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
		}

		// Receive and print data incoming from spi connection
		Message_struct_in message_in;
		if ( xQueueReceive(communespspi_in_queue_handler,  &message_in, 0) == pdTRUE ) {
			sprintf(str_out, "spi in: %u,%d, %d,%d, %d,%d, %d,%d, %d,%d \n\r\0",
					message_in.workmode, message_in.workmode_err,
					message_in.manctrly_perc, message_in.manctrly_err,
					message_in.manctrlx_perc, message_in.manctrlx_err,
					message_in.autctrl_speedy_mms, message_in.autctrl_speedy_err,
					message_in.autctrl_speedx_mms, message_in.autctrl_speedx_err
			);
			systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
		}

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
			message_out.ldist_mm = hcsr04_data[HCSR04_L_ID];
			message_out.rdist_err = false;
			message_out.rdist_mm = 150;
			xQueueSend(communespspi_out_queue_handler, &message_out, 0);

			char str_out_2[100];
			memset(str_out_2, 0, 100);
			sprintf(str_out_2, "spi out: %u,, %d,, %d,, %d,, %d,, %d,, %d,, %d,, %u,, %u, \n\r",
				message_out.workmode,
				message_out.manctrly_perc,
				message_out.manctrlx_perc,
				message_out.autctrl_speedy_mms,
				message_out.autctrl_speedx_mms,
				message_out.linspeed_mms,
				message_out.lspeed_rpm,
				message_out.rspeed_rpm,
				message_out.ldist_mm,
				message_out.rdist_mm
			);
			systools_transm_usart3((uint8_t *)str_out_2, strlen(str_out_2), 10);
		}

//		vTaskSuspend(NULL);
//		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
//		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_15);	// Toggle pin state

//		test_ts_diff_time();

		osDelay(1*configTICK_RATE_HZ/1000);
	}

}
*/


/*
 * motor_ctrl.c: gpio clock enable and timer enable
 *
 * Option 1: if / else
	if (motor->gpio_port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if (motor->gpio_port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (motor->gpio_port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if (motor->gpio_port == GPIOD)
		__HAL_RCC_GPIOD_CLK_ENABLE();
	else if (motor->gpio_port == GPIOE)
		__HAL_RCC_GPIOE_CLK_ENABLE();
	else if (motor->gpio_port == GPIOF)
		__HAL_RCC_GPIOF_CLK_ENABLE();
	else if (motor->gpio_port == GPIOG)
		__HAL_RCC_GPIOG_CLK_ENABLE();
	else if (motor->gpio_port == GPIOH)
		__HAL_RCC_GPIOH_CLK_ENABLE();
	else if (motor->gpio_port == GPIOI)
		__HAL_RCC_GPIOI_CLK_ENABLE();
	else if (motor->gpio_port == GPIOJ)
		__HAL_RCC_GPIOJ_CLK_ENABLE();
	else if (motor->gpio_port == GPIOK)
		__HAL_RCC_GPIOK_CLK_ENABLE();

 * Option 2: macros

#define PWM_L_PIN	GPIO_PIN_8
#define PWM_L_PORT_	B

//#define PWM_L_PORT	GPIOB
#define PASTER(x)	{ GPIO ## x }
#define EVALUATOR(x)	{ PASTER(x) }
#define PWM_L_PORT	EVALUATOR(PWM_L_PORT_)

#define PWM_L_CLK_EN()	__HAL_RCC_GPIOB_CLK_ENABLE()

#define TEMP	B		// GPIOB
#define PASTER(x) 		{ __HAL_RCC_GPIO ## x ## _CLK_ENABLE() ;}
#define EVALUATOR(x)  	{ PASTER(x) }

	GPIO_CLK_EN(PWM_L_PORT_);

 */
