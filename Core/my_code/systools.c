/******************************************************************************
 * @file tools_system.c
 * @brief File containing functions related to system tools (init, cache, gpio,
 * uart, etc.)
 *
 * \todo complete description
 ******************************************************************************/

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "systools.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <hcsr04.h>

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

static UART_HandleTypeDef huart3;
static bool uart_initialized = false;

/*******************************************************************************
 * FUNCTIONS PROTOTYPES
 ******************************************************************************/

static void SystemClock_Config(void);
static void MX_USART3_UART_Init(void);
static void MX_GPIO_Init(void);

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

/*****************************************
 * @brief System Clock Configuration
 * @retval None
 ****************************************/
static void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/* Configure LSE Drive Capability */
	HAL_PWR_EnableBkUpAccess();
	/* Configure the main internal regulator output voltage	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/* Activate the Over-Drive mode	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/* Initializes the CPU, AHB and APB buses clocks	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/*****************************************
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 ****************************************/
static void MX_USART3_UART_Init(void) {
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	uart_initialized = true;
}

/*****************************************
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 ****************************************/
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
//	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
//	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_TXD1_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
	GPIO_InitStruct.Pin = USB_SOF_Pin | USB_ID_Pin | USB_DM_Pin | USB_DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
	GPIO_InitStruct.Pin = RMII_TX_EN_Pin | RMII_TXD0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hcsr04_GPIO_init();

	/*Configure interr: USER_Btn_Pin, hcsr04_1_echo(D4/PF14) and hcsr04_2_echo(D7/PF12) */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 10);
	NVIC_SetPriorityGrouping( 0 );
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

/*****************************************
 * @brief System Clock Configuration
 * @retval None
 ****************************************/

void systools_hw_init(void) {
	/* Enable ICache and DCache */

	SCB_EnableICache();
	SCB_EnableDCache();

	/* Reset of all peripherals, Initializes Flash interface and Systick */
	HAL_Init();

	/* Configure the system clock. BUG in original code (file "stm32f7xx_hal_rcc.c" line 855):
	 * call "HAL_InitTick(TICK_INT_PRIORITY)" instead of "HAL_InitTick(uwTickPrio);" */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART3_UART_Init();
}

/*****************************************
 * @brief USART3 Transmit Function
 * @param None
 * @retval None
 ****************************************/
void systools_transm_usart3(uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	HAL_UART_Transmit(&huart3, pData, Size, Timeout);
}


/*****************************************
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 ****************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
}

/*****************************************
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 ****************************************/
void Error_Handler(void) {
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
}

#ifdef  USE_FULL_ASSERT
/*****************************************
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  ****************************************/
void assert_failed(uint8_t *file, uint32_t line) {
	if (uart_initialized == true)
	{
		char str_out[100];
		sprintf(str_out, "Wrong parameters value: file %s on line %lu\r\n", file, line);
		systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
	}
	Error_Handler();
}

/*****************************************
  * @brief Delay in us using timer10 (unsafe to use, since the timer will not
  * stop in case a context switch takes place in the middle)
  * @param time_us: time in us to delay. Max: 2^16
  * @retval None
  ****************************************/
/*
 * WARNING: TIM10 IS ALREADY BEING USED
void systools_delay_us_tim10(unsigned int time_us) {
	__HAL_TIM_SET_COUNTER(&htim10,0);
	HAL_TIM_Base_Start_IT(&htim10);
	while( __HAL_TIM_GET_COUNTER(&htim10) < time_us);
	HAL_TIM_Base_Stop_IT(&htim10);
}
*/

/*****************************************
  * @brief Delay in us using nops (safe to use; however, this only guarantees a minimum
  * delay, since a context switch in the middle would increase the delay time)
  * @param time_us: time in us to delay. Max: sizeof(unsigned int), typically 2^32
  * @retval None
  ****************************************/
void systools_delay_us_nops(unsigned int time_us) {
	#define NB_INSTR_PER_ITER	(1+3)	// 1 nop instr + 3 instr for loop control
	#define ITERS_PER_US		(HCLK_FREQ_MHZ*SIMULT_INSTR/NB_INSTR_PER_ITER)
										// 96MHz*2(dual_issue)/4instr_per_iter
	__asm__ __volatile__(
			".nop_init:" 		"\n\t"
//			"nop" 				"\r\n"	// We avoid using nop. According to the Cortex M7 references:
										// NOP does nothing. NOP is not necessarily a time-consuming NOP.
										//The processor might remove it from the pipeline before it reaches the execution stage
			"add	%[iter_count], #0"		"\r\n"	// Nop alternative
			"subs	%[iter_count], #1"		"\r\n"
			"cmp	%[iter_count], #0"		"\r\n"
			"bne.n	.nop_init"	"\r\n"
			: // No outputs
			: [iter_count] "r" (time_us*ITERS_PER_US)
			: // No clobbers
	);

	/*
	 * Alternative way (no control of object code generated)
		for (register unsigned int nb_nop=time_us*FREQ_CPU_MHZ; nb_nop>0; nb_nop--) {
			asm volatile ("nop");
		}
	*/
}


#endif /* USE_FULL_ASSERT */

