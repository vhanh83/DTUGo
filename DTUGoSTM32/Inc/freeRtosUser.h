#ifndef __FREE_RTOS_USER_H__
#define __FREE_RTOS_USER_H__



#include "main.h"
#include "cmsis_os.h"

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
void StartControlTask(void *argument);
void StartIMUTask(void *argument);
void StartRxCommTask(void *argument);
void StartTxCommTask(void *argument);
void StartBatteryTask(void *argument);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
#endif /* __FREE_RTOS_USER_H__ */