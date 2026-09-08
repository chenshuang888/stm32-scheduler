/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

/* USART1_TX 的 DMA 句柄（F407 上 USART1_TX 固定映射 DMA2_Stream7 / Channel 4）
   本工程未在 CubeMX 中启用 DMA，故手动声明并在 usart.c 的
   HAL_UART_MspInit() 中初始化 —— 发送侧需要它由 uart.c 引用。 */
extern DMA_HandleTypeDef hdma_usart1_tx;

/* USART1_RX 的 DMA 句柄（DMA2_Stream2 / Channel 4，循环模式）
   接收侧由 uart.c 用它启动循环接收，并读 NDTR 推算硬件写指针。 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

