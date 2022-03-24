/******************************************************************************
 * Project: Robocar
 * Application: on board app
 * Platform: STM Nucleo-F767ZI
 * @file communEspSpi.h
 * @version v1.0
 * @brief Header for communEspSpi.c file
******************************************************************************/

/*******************************************************************************
 * HEADER GUARD + EXTERN C: BEGIN
 ******************************************************************************/

#ifndef COMMUNESPSPI_H_
#define COMMUNESPSPI_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * INCLUSIONS
 ******************************************************************************/

#include "communMessages.h"

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

void communEspSpi_init(void);
void communEspSpi_task(void *argument);

/*******************************************************************************
 * HEADER GUARD + EXTERN C: END
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* COMMUNESPSPI_H_ */