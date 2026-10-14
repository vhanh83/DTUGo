/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


/* I2C1 init function */

/* USER CODE BEGIN 1 */

/**
 * I2C_Scan
 *  - Scans the I2C bus for 7-bit addresses 0x01..0x7F.
 *  - If `huart` is non-NULL, prints "I2C device found at 0xXX\r\n" to that UART.
 *  - At the end, prints "I2C scan complete: N device(s) found\r\n".
 *  - Returns HAL_OK (scanner itself always returns OK) — individual device readiness is printed.
 *
 * Note: HAL_I2C_IsDeviceReady uses 8-bit address (7-bit << 1).
 */
HAL_StatusTypeDef I2C_Scan(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    char msg[64];
    if (!hi2c) return HAL_ERROR;
    uint16_t found_count = 0;

    for (uint16_t addr = 1; addr < 128; ++addr) {
        /* HAL expects 8-bit address */
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 10) == HAL_OK) {
            if (huart) {
                int n = snprintf(msg, sizeof(msg), "I2C device found at 0x%02X\r\n", (int)addr);
                HAL_UART_Transmit(huart, (uint8_t*)msg, (uint16_t)n, PCA9685_I2C_TIMEOUT_MS);
            }
            found_count++;
        }
    }

    int n = snprintf(msg, sizeof(msg), "I2C scan complete: %u device(s) found\r\n", found_count);
    if (huart) {
        HAL_UART_Transmit(huart, (uint8_t*)msg, (uint16_t)n, PCA9685_I2C_TIMEOUT_MS);
    }

    return HAL_OK;
}

/* USER CODE END 1 */

