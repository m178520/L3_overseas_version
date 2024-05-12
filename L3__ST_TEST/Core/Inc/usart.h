/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define USART2_Max_Rxnum_size 15
#define USART2_Max_Txnum_size 15
#define USART2_Max_Rxbuf_size 512
#define USART2_Max_Txbuf_size 10

extern uint8_t USART2RxData[USART2_Max_Rxnum_size][USART2_Max_Rxbuf_size];
extern uint8_t USART2TxData[USART2_Max_Txnum_size][USART2_Max_Txbuf_size];

#define USART1_Max_Rxnum_size 15
#define USART1_Max_Txnum_size 15
#define USART1_Max_Rxbuf_size 700
#define USART1_Max_Txbuf_size 1500

extern uint8_t USART1RxData[USART1_Max_Rxnum_size][USART1_Max_Rxbuf_size];
extern uint8_t USART1TxData[USART1_Max_Txnum_size][USART1_Max_Txbuf_size];
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/* 串口设备结构体 */
typedef struct
{
	uint16_t usTxBufSize;		/* 接收缓冲区大小 */
	uint16_t usRxBufSize;		/* 接收缓冲区大小 */

	__IO uint16_t usTxWrite;	/* 发送缓冲区写指针 */
	__IO uint16_t usTxRead;		/* 发送缓冲区读指针 */
	__IO uint16_t usTxLen;	/* 还未读取的新数据个数 */
	
	__IO uint16_t usRxWrite;	/* 接收缓冲区写指针 */
	__IO uint16_t usRxRead;		/* 接收缓冲区读指针 */
}UART_fifo_t;

extern UART_fifo_t UART1_fifo;
extern UART_fifo_t UART2_fifo;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

