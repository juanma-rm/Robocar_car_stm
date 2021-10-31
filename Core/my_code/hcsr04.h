/******************************************************************************
 * @file hcsr04
 * @brief Header for hcsr04 file
 *
 * \todo complete description
 ******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef HCSR04_H_
#define HCSR04_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "stm32f7xx_hal.h"

/*******************************************************************************
 * USER
 ******************************************************************************/

#define NB_HCSR04	2

/* Sensor 0
 * - Trigger:	D2(PF15), output
 * - Echo:		D4(PF14), input, falling/rising interrupt
 */
#define HCSR04_0_TRIG_PIN GPIO_PIN_15
#define HCSR04_0_TRIG_PORT GPIOF
#define HCSR04_0_ECHO_PIN GPIO_PIN_14
#define HCSR04_0_ECHO_PORT GPIOF
/* Sensor 1
 * - Trigger:	D7(PF13), output
 * - Echo:		D8(PF12), input, falling/rising interrupt
 */
#define HCSR04_1_TRIG_PIN GPIO_PIN_13
#define HCSR04_1_TRIG_PORT GPIOF
#define HCSR04_1_ECHO_PIN GPIO_PIN_12
#define HCSR04_1_ECHO_PORT GPIOF

/*******************************************************************************
 * DEFINES
 ******************************************************************************/

#define TIM10_PRESC		96
#define TIM10_FREQ_HZ	(HCLK_FREQ_HZ/TIM10_PRESC)	// 1 us (for HCLK=96MHz)
#define TIM10_FREQ_MHZ	(TIM10_FREQ_HZ/MHZ_TO_HZ)

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

void hcsr04_GPIO_init(void);
void hcsr04_TIM10_init(void);
void hcsr04_semaph_queues_init(void);
void hcsr04_Task(void *argument);
void hcsr04_ISR(unsigned int hcsr04_index);

/*******************************************************************************
 * FUNCTION DEFINITIONS
 ******************************************************************************/




/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HCSR04_H_ */
