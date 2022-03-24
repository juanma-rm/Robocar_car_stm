/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file communEspSpi.c
 * @version v1.0
 *
 * @brief File containing functions related to the communication with the esp
 * device via SPI interface. The message formats are defined externally in
 * communMessages.h. Besides, an ending value (uint16_t 0x0E0F) is sent so that
 * the master device can distinguish when real data is being sent (all 0's in
 * MISO line would be otherwise interpreted as a valid message)
 *
 * Requirements:
 *	0.1) SPI configuration:
 *		stm32f7xx_hal_conf.h:
 *		- #define HAL_SPI_MODULE_ENABLED
 *		STM32F7xx_HAL_Driver\inc and STM32F7xx_HAL_Driver\Src:
 *		- To add stm32f7xx_hal_spi.h/c and stm32f7xx_hal_spi_ex.h/c
 *	0.2) 10k pull-down in SCLK (for mode polarity:low / 1st edge) and 10k pull-up in CS
 *	1) Call communEspSpi_init()
 * 	2) Initialize task from main: using communEspSpi_Task() as task function
 *
 * Warnings!
 *  1) SPI1 pins are used here
 *****************************************************************************/

//#define DEBUG_SPI		// Uncomment for debugging (serial output)

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

// Std
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#if defined(DEBUG_SPI)
#include <stdio.h>
#endif
// Freertos and hardware
#include "stm32f7xx_hal.h"
#include "cmsis_os.h"
#include "queue.h"
// Project
#include "communEspSpi.h"
#include "systools.h"

/*******************************************************************************
 * DEFINES
 ******************************************************************************/

/* Exported macro */
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_3
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

/*******************************************************************************
 * DATA STRUCTS / ENUMS
 ******************************************************************************/

enum spi_state {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Task handle / attributes */
osThreadId_t communEspSpi_taskHandle;
const osThreadAttr_t communEspSpi_taskAttr = {
		.name = "communEspSpi_task",
		.stack_size = 128 * 8,
		.priority = (osPriority_t) osPriorityNormal,
};

/* Semaphores and queues definitions */

#define COMMUNESPSPI_IN_QUEUE_LENGTH	1
xQueueHandle communespspi_in_queue_handler;
static StaticQueue_t communespspi_in_queue_buffer;
static uint8_t communespspi_in_queue_stbuff[COMMUNESPSPI_IN_QUEUE_LENGTH
		* sizeof(Message_struct_in) / sizeof(uint8_t)];
		// the queue will store Message_struct_in type elements

#define COMMUNESPSPI_OUT_QUEUE_LENGTH	1
xQueueHandle communespspi_out_queue_handler;
static StaticQueue_t communespspi_out_queue_buffer;
static uint8_t communespspi_out_queue_stbuff[COMMUNESPSPI_OUT_QUEUE_LENGTH
		* sizeof(Message_struct_out) / sizeof(uint8_t)];
		// the queue will store Message_struct_out type elements

/*******************************************************************************
 * PRIVATE VARIABLES
 ******************************************************************************/

static SPI_HandleTypeDef spi_handle;
volatile enum spi_state comm_state = TRANSFER_WAIT;
const uint16_t end_of_sequence = 0x0E0F;

/* Debug */
#if defined(DEBUG_SPI)
static char str_out[100];
#endif

/*******************************************************************************
 * FUNCTIONS PROTOTYPES: PRIVATE
 ******************************************************************************/

static void spi_reset(SPI_HandleTypeDef *hspi);
static bool decode_params_recv(uint16_t *rxBuffer_ptr, Message_struct_in *decoded_ptr);
static void encode_params_trans(uint16_t *txBuffer_ptr, Message_struct_out *decoded_ptr);

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PRIVATE
 ******************************************************************************/

/*****************************************
 * @brief reset spi peripheral
 * @note taken from
 * https://www.eevblog.com/forum/microcontrollers/stm32-spi-slave-inconsistent/
 ****************************************/
static void spi_reset(SPI_HandleTypeDef *hspi) {
	if((hspi->Instance->SR & SPI_SR_BSY) || (hspi->State!=HAL_SPI_STATE_READY)){	// If peripheral is actually busy or handler not ready
		hspi->State=HAL_SPI_STATE_RESET;					// Force reset state (HAL_SPI_Init will fail if not in reset state)

		SPIx_FORCE_RESET();									// Reset SPI
		asm("nop\nnop\nnop\nnop");							// Wait few clocks just in case
		while(hspi->Instance->SR & SPI_SR_BSY);				// Wait until Busy is gone
		SPIx_RELEASE_RESET();								// Release reset
		asm("nop\nnop\nnop\nnop");							// Wait few clocks just in case
		while(hspi->Instance->SR & SPI_SR_BSY);				// Check again
		if (HAL_SPI_Init(hspi) != HAL_OK){ Error_Handler(); }	// Re-init SPI
	}
}

/*****************************************
 * @brief decode the raw content of an incoming message pointed by rxBuffer_ptr,
 * checking whether each parameters is in range (enabling its error field otherwise)
 * @param rxBuffer_ptr is the uint16_t pointer to the memory where the raw
 * message is stored
 * @param decoded_ptr is the Message_struct_in pointer where the decoded
 * message will be stored
 * @retval message_received is true if a message was properly received
 ****************************************/
static bool decode_params_recv(	uint16_t *rxBuffer_ptr,
								Message_struct_in *decoded_ptr) {
	bool message_received = false;
	if (rxBuffer_ptr[MESSAGE_IN_NB_FIELDS] == end_of_sequence) {
		message_received = true;
		for (unsigned int iter_buff=0; iter_buff<MESSAGE_IN_NB_FIELDS; ++iter_buff) {
			switch (iter_buff) {
			case MESSAGE_IN_POS_WORKMODE:
				decoded_ptr->workmode = (uint16_t) rxBuffer_ptr[iter_buff];
				decoded_ptr->workmode_err = (decoded_ptr->workmode != 0 &&
											 decoded_ptr->workmode != 1 &&
											 decoded_ptr->workmode != 2		);
				break;
			case MESSAGE_IN_POS_MANCTRLY_PERC:
				decoded_ptr->manctrly_perc = (int16_t) rxBuffer_ptr[iter_buff];
				decoded_ptr->manctrly_err = (decoded_ptr->manctrly_perc < -100 ||
											 decoded_ptr->manctrly_perc > 100);
				break;
			case MESSAGE_IN_POS_MANCTRLX_PERC:
				decoded_ptr->manctrlx_perc = (int16_t) rxBuffer_ptr[iter_buff];
				decoded_ptr->manctrlx_err = (decoded_ptr->manctrlx_perc == INT16_MIN);
				break;
			case MESSAGE_IN_POS_AUTCTRL_SPEEDY_MMS:
				decoded_ptr->autctrl_speedy_mms = (int16_t) rxBuffer_ptr[iter_buff];
				decoded_ptr->autctrl_speedy_err = (decoded_ptr->autctrl_speedy_mms == INT16_MIN);
				break;
			case MESSAGE_IN_POS_AUTCTRL_SPEEDX_MMS:
				decoded_ptr->autctrl_speedx_mms = (int16_t) rxBuffer_ptr[iter_buff];
				decoded_ptr->autctrl_speedx_err = (decoded_ptr->autctrl_speedx_mms == INT16_MIN);
				break;
			default:
				break;
			}
		}
	}
	return message_received;
}

/*****************************************
 * @brief encode an outgoing message pointed by txBuffer_ptr to be sent
 * @param txBuffer_ptr is the uint16_t pointer to the memory where the raw
 * message to be sent is stored
 * @param decoded_ptr points to the Message_struct_out variable where the
 * decoded data to be sent is stored
 ****************************************/
static void encode_params_trans(uint16_t *txBuffer_ptr, Message_struct_out *decoded_ptr) {
	for (unsigned int iter_buff=0; iter_buff<MESSAGE_OUT_NB_FIELDS; ++iter_buff) {
		switch (iter_buff) {
		case MESSAGE_OUT_POS_WORKMODE:
			txBuffer_ptr[iter_buff] = (decoded_ptr->workmode_err == true) ?
				UINT16_MAX : decoded_ptr->workmode;
			break;
		case MESSAGE_OUT_POS_MANCTRLY_PERC:
			txBuffer_ptr[iter_buff] = (decoded_ptr->manctrly_err == true) ?
				INT16_MIN : decoded_ptr->manctrly_perc;
			break;
		case MESSAGE_OUT_POS_MANCTRLX_PERC:
			txBuffer_ptr[iter_buff] = (decoded_ptr->manctrlx_err == true) ?
				INT16_MIN : decoded_ptr->manctrlx_perc;
			break;
		case MESSAGE_OUT_POS_AUTCTRL_SPEEDY_MMS:
			txBuffer_ptr[iter_buff] = (decoded_ptr->autctrl_speedy_err == true) ?
				INT16_MIN : decoded_ptr->autctrl_speedy_mms;
			break;
		case MESSAGE_OUT_POS_AUTCTRL_SPEEDX_MMS:
			txBuffer_ptr[iter_buff] = (decoded_ptr->autctrl_speedx_err == true) ?
				INT16_MIN : decoded_ptr->autctrl_speedx_mms;
			break;
		case MESSAGE_OUT_POS_LINSPEED_MMS:
			txBuffer_ptr[iter_buff] = (decoded_ptr->linspeed_err == true) ?
				INT16_MIN : decoded_ptr->linspeed_mms;
			break;
		case MESSAGE_OUT_POS_LSPEED_RPM:
			txBuffer_ptr[iter_buff] = (decoded_ptr->lspeed_err == true) ?
				INT16_MIN : decoded_ptr->lspeed_rpm;
			break;
		case MESSAGE_OUT_POS_RSPEED_RPM:
			txBuffer_ptr[iter_buff] = (decoded_ptr->rspeed_err == true) ?
				INT16_MIN : decoded_ptr->rspeed_rpm;
			break;
		case MESSAGE_OUT_POS_LDIST_MM:
			txBuffer_ptr[iter_buff] = (decoded_ptr->ldist_err == true) ?
				INT16_MIN : decoded_ptr->ldist_mm;
			break;
		case MESSAGE_OUT_POS_RDIST_MM:
			txBuffer_ptr[iter_buff] = (decoded_ptr->rdist_err == true) ?
				INT16_MIN : decoded_ptr->rdist_mm;
			break;
		default:
			break;
		}
	}
	// End of sequence
	txBuffer_ptr[MESSAGE_OUT_NB_FIELDS] = end_of_sequence;
}

/*******************************************************************************
 * FUNCTIONS DEFINITIONS: PUBLIC
 ******************************************************************************/

/*****************************************
 * @brief init SPI hardware and queues
 ****************************************/
void communEspSpi_init(void) {

	spi_handle.Instance               = SPIx;
	spi_handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	spi_handle.Init.Direction         = SPI_DIRECTION_2LINES;
	spi_handle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	spi_handle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	spi_handle.Init.DataSize          = SPI_DATASIZE_16BIT;
	spi_handle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	spi_handle.Init.TIMode            = SPI_TIMODE_DISABLE;
	spi_handle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	spi_handle.Init.CRCPolynomial     = 7;
	spi_handle.Init.NSS               = SPI_NSS_SOFT;
	spi_handle.Init.Mode = SPI_MODE_SLAVE;
	if (HAL_SPI_Init(&spi_handle) != HAL_OK) {
		Error_Handler();
	}

	communespspi_in_queue_handler = xQueueCreateStatic(
			COMMUNESPSPI_IN_QUEUE_LENGTH, sizeof(Message_struct_in),
			communespspi_in_queue_stbuff, &(communespspi_in_queue_buffer));
	communespspi_out_queue_handler = xQueueCreateStatic(
			COMMUNESPSPI_OUT_QUEUE_LENGTH, sizeof(Message_struct_out),
			communespspi_out_queue_stbuff, &(communespspi_out_queue_buffer));
}

/*****************************************
 * @brief Task in charge of encode message to send, wait for SPI communication
 * to send and receive and decode incoming message
 * @param argument = NULL (not used)
 ****************************************/
void communEspSpi_task(void *argument) {

	// The SPI transaction (in both directions) will be as long as the maximum among
	// fields to receive and to send, plus 1 (to indicate end of sequence)
	unsigned int spi_trans_length = max(MESSAGE_IN_NB_FIELDS,MESSAGE_OUT_NB_FIELDS) + 1;
	Message_struct_in decoded_in_mess;
	Message_struct_out decoded_out_mess;
	uint16_t txBuffer[spi_trans_length];
	uint16_t rxBuffer[spi_trans_length];

	while(1) {

		// If there is data to send out via SPI: encode and place it in txBuffer to be sent. Otherwise: send 0s
		memset(txBuffer, 0, sizeof(txBuffer));
		if ( xQueueReceive(communespspi_out_queue_handler,  &decoded_out_mess, 0) == pdTRUE ) {
			encode_params_trans(txBuffer, &decoded_out_mess);
		}
		if(HAL_SPI_TransmitReceive_IT(&spi_handle, (uint8_t*)txBuffer, (uint8_t *)rxBuffer, spi_trans_length) == HAL_OK) {
			while (comm_state == TRANSFER_WAIT)	{}	// Wait for the end of the transfer
			if (comm_state == TRANSFER_COMPLETE) {
				if (decode_params_recv(rxBuffer, &decoded_in_mess) == true) {
					xQueueSendToBack(communespspi_in_queue_handler, &decoded_in_mess, 0);
				}
				comm_state = TRANSFER_WAIT;	// Ready for next transfer
#if defined(DEBUG_SPI)
				memset(str_out, 0, 100);
				sprintf(str_out, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,\n\r",
						txBuffer[0],txBuffer[1],txBuffer[2],txBuffer[3],txBuffer[4],
						txBuffer[5],txBuffer[6],txBuffer[7],txBuffer[8],txBuffer[9],
						txBuffer[10]
				);
				systools_transm_usart3((uint8_t *)str_out, strlen(str_out), 10);
#endif
			} else {
				spi_reset(&spi_handle);
			}
		} else {
			spi_reset(&spi_handle);
		}
	}
}

/*****************************************
 * @brief  TxRx Transfer completed callback
 * @param  hspi: SPI handle
 ****************************************/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	comm_state = TRANSFER_COMPLETE;
}

/*****************************************
 * @brief  SPI error callback
 * @param  hspi: SPI handle
 ****************************************/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	comm_state = TRANSFER_ERROR;
}

/*****************************************
 * @brief  This function handles SPI interrupt request.
 ****************************************/
void SPIx_IRQHandler(void) {
	HAL_SPI_IRQHandler(&spi_handle);
}

/*****************************************
 * @brief  SPI MSP Initialization (SPI hardware resources)
 * @param  hspi: SPI handle
 ****************************************/
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef  GPIO_InitStruct;

	if (hspi->Instance == SPIx)
	{
		/* 1) Enable peripherals and GPIO Clocks */
		/* Enable GPIO TX/RX clock */
		SPIx_SCK_GPIO_CLK_ENABLE();
		SPIx_MISO_GPIO_CLK_ENABLE();
		SPIx_MOSI_GPIO_CLK_ENABLE();
		/* Enable SPI clock */
		SPIx_CLK_ENABLE();

		/* 2) Configure peripheral GPIO */
		/* SPI SCK GPIO pin configuration  */
		GPIO_InitStruct.Pin       = SPIx_SCK_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = SPIx_SCK_AF;
		HAL_GPIO_Init(SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

		/* SPI MISO GPIO pin configuration  */
		GPIO_InitStruct.Pin = SPIx_MISO_PIN;
		GPIO_InitStruct.Alternate = SPIx_MISO_AF;
		HAL_GPIO_Init(SPIx_MISO_GPIO_PORT, &GPIO_InitStruct);

		/* SPI MOSI GPIO pin configuration  */
		GPIO_InitStruct.Pin = SPIx_MOSI_PIN;
		GPIO_InitStruct.Alternate = SPIx_MOSI_AF;
		HAL_GPIO_Init(SPIx_MOSI_GPIO_PORT, &GPIO_InitStruct);

		/* 3) Configure the NVIC for SPI */
		/* NVIC for SPI */
		HAL_NVIC_SetPriority(SPIx_IRQn, 1, 0);
		HAL_NVIC_EnableIRQ(SPIx_IRQn);
	}
}

/*****************************************
 * @brief	SPI MSP De-Initialization: free the hardware resources
 * @param 	hspi: SPI handle pointer
 * @retval None
 ****************************************/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if(hspi->Instance == SPIx)
	{
		/* 1) Reset peripherals */
		SPIx_FORCE_RESET();
		SPIx_RELEASE_RESET();

		/* 2) Disable peripherals and GPIO Clocks */
		/* Configure SPI SCK as alternate function  */
		HAL_GPIO_DeInit(SPIx_SCK_GPIO_PORT, SPIx_SCK_PIN);
		/* Configure SPI MISO as alternate function  */
		HAL_GPIO_DeInit(SPIx_MISO_GPIO_PORT, SPIx_MISO_PIN);
		/* Configure SPI MOSI as alternate function  */
		HAL_GPIO_DeInit(SPIx_MOSI_GPIO_PORT, SPIx_MOSI_PIN);

		/* 3) Disable the NVIC for SPI */
		HAL_NVIC_DisableIRQ(SPIx_IRQn);
	}
}

