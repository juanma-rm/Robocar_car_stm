/******************************************************************************
 * @file tools_system.h
 * @brief Header for tools_system.c file
 *
 * \todo complete description
 ******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef TOOLS_SYSTEM_H_
#define TOOLS_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "stm32f7xx_hal.h"

/*******************************************************************************
 * DEFINES AND CONST DATA
 ******************************************************************************/

#define MHZ_TO_HZ		(1000*1000)

#define HCLK_FREQ_MHZ	(96)	// HCLK = 96 MHz (to adapt for different HCLK)
#define HCLK_FREQ_HZ	(HCLK_FREQ_MHZ*MHZ_TO_HZ)
#define SIMULT_INSTR	2	// "The processor has an in-order super-scalar pipeline
							// that means many instructions can be dual-issued".
#define FREQ_PER_INST_HZ	(HCLK_FREQ_HZ*SIMULT_INSTR)
#define FREQ_PER_INST_MHZ	(FREQ_PER_INST_HZ/MHZ_TO_HZ)

// This is finally not used
//#define CPUCLK_FROM_HCLK	(8)			/* The RCC feeds the external clock of
//	the Cortex System Timer (SysTick) with the AHB clock (HCLK) divided by 8
//	(ref: stm32_STM32F76xxx and STM32F77xxx advanced_RM0410) */
//#define FREQ_CPU_MHZ	( HCLK_FREQ_HZ / (CPUCLK_FROM_HCLK*MHZ_TO_HZ) )



/************************************************
 * Pinout (ST NUCLEO-F767ZI / STM32F767ZI)
 ***********************************************/

#define USER_Btn_Pin 			GPIO_PIN_13
#define USER_Btn_GPIO_Port 		GPIOC
#define MCO_Pin 				GPIO_PIN_0
#define MCO_GPIO_Port 			GPIOH
#define RMII_MDC_Pin 			GPIO_PIN_1
#define RMII_MDC_GPIO_Port 		GPIOC
#define RMII_REF_CLK_Pin 		GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port 	GPIOA
#define RMII_MDIO_Pin 			GPIO_PIN_2
#define RMII_MDIO_GPIO_Port 	GPIOA
#define RMII_CRS_DV_Pin 		GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port 	GPIOA
#define RMII_RXD0_Pin 			GPIO_PIN_4
#define RMII_RXD0_GPIO_Port 	GPIOC
#define RMII_RXD1_Pin 			GPIO_PIN_5
#define RMII_RXD1_GPIO_Port 	GPIOC
#define LD1_Pin 				GPIO_PIN_0
#define LD1_GPIO_Port 			GPIOB
#define RMII_TXD1_Pin 			GPIO_PIN_13
#define RMII_TXD1_GPIO_Port 	GPIOB
#define LD3_Pin 				GPIO_PIN_14
#define LD3_GPIO_Port 			GPIOB
#define STLK_RX_Pin 			GPIO_PIN_8
#define STLK_RX_GPIO_Port 		GPIOD
#define STLK_TX_Pin 			GPIO_PIN_9
#define STLK_TX_GPIO_Port 		GPIOD
#define USB_PowerSwitchOn_Pin 	GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port 	GPIOG
#define USB_OverCurrent_Pin 	GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port 		GPIOG
#define USB_SOF_Pin 			GPIO_PIN_8
#define USB_SOF_GPIO_Port 		GPIOA
#define USB_VBUS_Pin 			GPIO_PIN_9
#define USB_VBUS_GPIO_Port 		GPIOA
#define USB_ID_Pin 				GPIO_PIN_10
#define USB_ID_GPIO_Port 		GPIOA
#define USB_DM_Pin 				GPIO_PIN_11
#define USB_DM_GPIO_Port 		GPIOA
#define USB_DP_Pin 				GPIO_PIN_12
#define USB_DP_GPIO_Port 		GPIOA
#define TMS_Pin 				GPIO_PIN_13
#define TMS_GPIO_Port 			GPIOA
#define TCK_Pin 				GPIO_PIN_14
#define TCK_GPIO_Port 			GPIOA
#define RMII_TX_EN_Pin 			GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port 	GPIOG
#define RMII_TXD0_Pin 			GPIO_PIN_13
#define RMII_TXD0_GPIO_Port 	GPIOG
#define SWO_Pin 				GPIO_PIN_3
#define SWO_GPIO_Port 			GPIOB
#define LD2_Pin 				GPIO_PIN_7
#define LD2_GPIO_Port 			GPIOB

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif /* USE_FULL_ASSERT */
void Error_Handler(void);
void systools_hw_init(void);
void systools_transm_usart3(uint8_t *pData, uint16_t Size, uint32_t Timeout);
void systools_delay_us_nops(unsigned int time_us);
void systools_delay_us_tim10(unsigned int time_us);

/*******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************/

/*****************************************
 * @brief
 * @retval None
 ****************************************/
#define systools_diff_time(t1,t2,tmax) ( (t1 < t2) ? (t2-t1) : (t2+tmax-t1) )


/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* TOOLS_SYSTEM_H_ */
