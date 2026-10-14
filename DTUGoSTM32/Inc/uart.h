#ifndef __UART_H
#define __UART_H
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_RX_SIZE 128				//gioi han so ky tu moi lan nhan la 20
extern uint8_t data_RX[UART_RX_SIZE];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
void SerialPrintfU2(const char *fmt, ...);
void SerialPrintfU6(const char *fmt, ...);

#endif