/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */
// UART Debug functions
void UART_Printf(const char* format, ...);
void UART_SendString(const char* str);
void UART_SendChar(char c);
void UART_NewLine(void);
void UART_ClearScreen(void);

// MPU6050 Display functions
void UART_DisplayMPU6050Data(void);
void UART_DisplayAccelerometer(float ax, float ay, float az);
void UART_DisplayGyroscope(float gx, float gy, float gz);
void UART_DisplayTemperature(float temp);
void UART_DisplayAngles(float roll, float pitch, float yaw);

// printf support
int _write(int file, char *ptr, int len);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

