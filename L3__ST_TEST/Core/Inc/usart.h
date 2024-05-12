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
/* �����豸�ṹ�� */
typedef struct
{
	uint16_t usTxBufSize;		/* ���ջ�������С */
	uint16_t usRxBufSize;		/* ���ջ�������С */

	__IO uint16_t usTxWrite;	/* ���ͻ�����дָ�� */
	__IO uint16_t usTxRead;		/* ���ͻ�������ָ�� */
	__IO uint16_t usTxLen;	/* ��δ��ȡ�������ݸ��� */
	
	__IO uint16_t usRxWrite;	/* ���ջ�����дָ�� */
	__IO uint16_t usRxRead;		/* ���ջ�������ָ�� */
}UART_fifo_t;

extern UART_fifo_t UART1_fifo;
extern UART_fifo_t UART2_fifo;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

