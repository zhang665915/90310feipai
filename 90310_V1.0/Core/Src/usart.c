/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "usart.h"
#include "crc.h"
#include "tjshow.h"
#include "encoder.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
/* USER CODE BEGIN 0 */


#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart6, (uint8_t*)&ch,1,HAL_MAX_DELAY);
	    return ch;
}

uint8_t RxData[5] ;

/* USER CODE END 0 */
UART_HandleTypeDef huart1;		// 用于串口屏通信
UART_HandleTypeDef huart2;		//RS232用，上传合成速度给工控机
//UART_HandleTypeDef huart3;
//UART_HandleTypeDef huart4;
//UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;		//暂时用于测试打印
//UART_HandleTypeDef huart7;
//UART_HandleTypeDef huart8;

void MX_USART1_UART_Init( uint32_t bound )
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = bound;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*)TJC_RxData, SIZEOF_TJC_RxData );
}

/* USART2 init function */

void MX_USART2_UART_Init( uint32_t bound )
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = bound ;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_UART_Receive_IT( &huart2, (uint8_t*)RxData, 5 );
}


/* USART3 init function */
//void MX_USART3_UART_Init( uint32_t bound )
//{
//  huart3.Instance = USART3;
//  huart3.Init.BaudRate = bound;
//  huart3.Init.WordLength = UART_WORDLENGTH_8B;
//  huart3.Init.StopBits = UART_STOPBITS_1;
//  huart3.Init.Parity = UART_PARITY_NONE;
//  huart3.Init.Mode = UART_MODE_TX_RX;
//  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  HAL_UART_Receive_IT( &huart3, (uint8_t*) TJC_RxData, SIZEOF_TJC_RxData );
//}


/* UART4 init function */
//void MX_USART4_UART_Init(uint32_t bound)
//{
//  huart4.Instance = UART4;
//  huart4.Init.BaudRate = bound;
//  huart4.Init.WordLength = UART_WORDLENGTH_8B;
//  huart4.Init.StopBits = UART_STOPBITS_1;
//  huart4.Init.Parity = UART_PARITY_NONE;
//  huart4.Init.Mode = UART_MODE_TX_RX;
//  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*) TJC_RxData, SIZEOF_TJC_RxData ) ;

//}

//void MX_USART5_UART_Init(uint32_t bound)
//{
//  huart5.Instance = UART5;
//  huart5.Init.BaudRate = bound;
//  huart5.Init.WordLength = UART_WORDLENGTH_8B;
//  huart5.Init.StopBits = UART_STOPBITS_1;
//  huart5.Init.Parity = UART_PARITY_NONE;
//  huart5.Init.Mode = UART_MODE_TX_RX;
//  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

void MX_USART6_UART_Init(uint32_t bound)
{
  huart6.Instance = USART6;
  huart6.Init.BaudRate = bound;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
}

//void MX_USART7_UART_Init(uint32_t bound)
//{
//
//  huart7.Instance = UART7;
//  huart7.Init.BaudRate = bound;
//  huart7.Init.WordLength = UART_WORDLENGTH_8B;
//  huart7.Init.StopBits = UART_STOPBITS_1;
//  huart7.Init.Parity = UART_PARITY_NONE;
//  huart7.Init.Mode = UART_MODE_TX_RX;
//  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
//void MX_USART8_UART_Init(uint32_t bound)
//{
//
//  huart8.Instance = UART8;
//  huart8.Init.BaudRate = bound;
//  huart8.Init.WordLength = UART_WORDLENGTH_8B;
//  huart8.Init.StopBits = UART_STOPBITS_1;
//  huart8.Init.Parity = UART_PARITY_NONE;
//  huart8.Init.Mode = UART_MODE_TX_RX;
//  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
//  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}


void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_ENABLE();

    __USART1_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART1_TX_PIN|USART1_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(USART1_GPIO_PORT, &GPIO_InitStruct);

	/* 485控制引脚配置 */
//	GPIO_InitStruct.Pin = USART1_RS485_EN_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_SSOUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(USART1_RS485_EN_GPIO_PORT, &GPIO_InitStruct);

//	USART1_485_EN(0);		//开启接收模式

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

  }

  else if(uartHandle->Instance==USART2)
  {
    __HAL_RCC_USART2_CLK_ENABLE();

    __USART2_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART2_TX_PIN|USART2_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(USART2_GPIO_PORT, &GPIO_InitStruct);

	/* 485控制引脚配置 */
//	GPIO_InitStruct.Pin = USART2_RS485_EN_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(USART2_RS485_EN_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

//    F4016_8(0);							//默认设置485为接收模式
  }

//  else if(uartHandle->Instance==USART3)
//  {
//    __HAL_RCC_USART3_CLK_ENABLE();
//
//    __USART3_GPIO_CLK_ENABLE();
//
//    GPIO_InitStruct.Pin = USART3_TX_PIN|USART3_RX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//    HAL_GPIO_Init(USART3_GPIO_PORT, &GPIO_InitStruct);
//
//	/* 485控制引脚配置 */
//	GPIO_InitStruct.Pin = USART3_RS485_EN_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(USART3_RS485_EN_GPIO_PORT, &GPIO_InitStruct);
//
//    HAL_NVIC_SetPriority(USART3_IRQn,7, 0);
//    HAL_NVIC_EnableIRQ(USART3_IRQn);
//
////    USART3_485_EN(0);
//
//  }

//  else if(uartHandle->Instance==UART4)
//  {
//    __HAL_RCC_UART4_CLK_ENABLE();
//
//    __USART4_GPIO_CLK_ENABLE();
//
//    GPIO_InitStruct.Pin = USART4_TX_PIN|USART4_RX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
//    HAL_GPIO_Init(USART4_GPIO_PORT, &GPIO_InitStruct);
//
//    HAL_NVIC_SetPriority(UART4_IRQn, 7, 0);
//    HAL_NVIC_EnableIRQ(UART4_IRQn);
//  }

  else if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_ENABLE();

    __USART6_GPIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = USART6_TX_PIN|USART6_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART6;
    HAL_GPIO_Init(USART6_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART6_IRQn, 8, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

  }

//  else if(uartHandle->Instance==UART7)
//  {
//    __HAL_RCC_UART7_CLK_ENABLE();
//
//    __USART7_GPIO_CLK_ENABLE();
//
//    GPIO_InitStruct.Pin = USART7_TX_PIN|USART7_RX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF7_UART7;
//    HAL_GPIO_Init(USART7_GPIO_PORT, &GPIO_InitStruct);
//
//    HAL_NVIC_SetPriority(UART7_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(UART7_IRQn);
//  }

//  else if(uartHandle->Instance==UART8)
//  {
//
//    __HAL_RCC_UART8_CLK_ENABLE();
//    __USART8_GPIO_CLK_ENABLE();
//
//    GPIO_InitStruct.Pin = USART8_TX_PIN|USART8_RX_PIN;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
//    HAL_GPIO_Init(USART8_GPIO_PORT, &GPIO_InitStruct);
//
//	/* 485控制引脚配置 */
//	GPIO_InitStruct.Pin = USART8_RS485_EN_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(USART8_RS485_EN_GPIO_PORT, &GPIO_InitStruct);
//	HAL_GPIO_WritePin(USART8_RS485_EN_GPIO_PORT, USART8_RS485_EN_PIN, 0);		//开启接收模式
//
//    HAL_NVIC_SetPriority(UART8_IRQn, 8, 0);
//    HAL_NVIC_EnableIRQ(UART8_IRQn);
//  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
    __HAL_RCC_USART1_CLK_DISABLE();
  
    HAL_GPIO_DeInit(USART1_GPIO_PORT, USART1_TX_PIN|USART1_RX_PIN);

    HAL_NVIC_DisableIRQ(USART1_IRQn);

  }

  else if(uartHandle->Instance==USART2)
  {

    __HAL_RCC_USART2_CLK_DISABLE();

    HAL_GPIO_DeInit(USART2_GPIO_PORT, USART2_TX_PIN|USART2_RX_PIN);

    HAL_NVIC_DisableIRQ(USART2_IRQn);

  }

//  else if(uartHandle->Instance==USART3)
//  {
//
//    __HAL_RCC_USART3_CLK_DISABLE();
//
//    HAL_GPIO_DeInit(USART3_GPIO_PORT, USART3_TX_PIN|USART3_RX_PIN);
//
//    HAL_NVIC_DisableIRQ(USART3_IRQn);
//
//  }
//
//  else if(uartHandle->Instance==UART4)
//  {
//    __HAL_RCC_UART4_CLK_DISABLE();
//
//    HAL_GPIO_DeInit(USART4_GPIO_PORT, USART4_TX_PIN|USART4_RX_PIN);
//
//    HAL_NVIC_DisableIRQ(UART4_IRQn);
//
//  }

  else if(uartHandle->Instance==USART6)
  {
    __HAL_RCC_USART6_CLK_DISABLE();

    HAL_GPIO_DeInit(USART6_GPIO_PORT, USART6_TX_PIN|USART6_RX_PIN);

    HAL_NVIC_DisableIRQ(USART6_IRQn);

  }

//  else if(uartHandle->Instance==UART7)
//  {
//    __HAL_RCC_UART7_CLK_DISABLE();
//
//    HAL_GPIO_DeInit(USART7_GPIO_PORT, USART7_TX_PIN|USART7_RX_PIN);
//
//    HAL_NVIC_DisableIRQ(UART7_IRQn);
//  }

//  else if(uartHandle->Instance==UART8)
//  {
//    __HAL_RCC_UART8_CLK_DISABLE();
//
//    HAL_GPIO_DeInit(USART8_GPIO_PORT, USART8_TX_PIN|USART8_RX_PIN);
//
//    HAL_NVIC_DisableIRQ(UART8_IRQn);
//  }
} 


/*****************************************************************
 * @brief  HAL_UART_RxCpltCallback
 *         USART1 接收中断回调函数
   *         接收串口屏数据
 * @param
 * @retval
 * @note
 ****************************************************************/


void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
//	uint8_t Sum = 0;
//	uint16_t crc16 = 0xFFFF;

	if ( huart->Instance == huart1.Instance )
	{
		if( TJC_RxData[0] == 0xAA && TJC_RxData[1] == 0xBB && TJC_RxData[2] == 0xCC )	//AABBCC是数据包头，校验作用
		{
			xEventGroupSetBits(Rx_Set_Freq_Event, RX_SET_FREQ_EVENT);
//			Set_Freq_Flag = 1;	//设置频率标志位置1，在 Tjshow_Task 任务里处理数据并回显给串口屏
		}
		else
		{
			memset(TJC_RxData, 0, SIZEOF_TJC_RxData);
			MX_USART1_UART_Init(BOUND_115200);
		}
//		HAL_UART_AbortReceive(TJHMI_HUART);  //接收复位
		HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*)TJC_RxData, SIZEOF_TJC_RxData );
	}

	else if ( huart->Instance == huart2.Instance )										// 8 通道接收中断
	{

		HAL_UART_Receive_IT( &huart2, RxData, 5 );
	}

//	else if ( huart->Instance == huart3.Instance )									// 12 通道接收中断
//	{
//		if( TJC_RxData[0] == 0xAA && TJC_RxData[1] == 0xBB && TJC_RxData[2] == 0xCC )	//AABBCC是数据包头，校验作用
//		{
//			Set_Freq_Flag = 1;	//设置频率标志位置1，在 Tjshow_Task 任务里处理数据并回显给串口屏
//		}
//		else
//		{
//			memset(TJC_RxData, 0, SIZEOF_TJC_RxData);
//			MX_USART3_UART_Init();
//		}
//
//		HAL_UART_Receive_IT( &huart3, (uint8_t*) TJC_RxData, SIZEOF_TJC_RxData ) ;
//		crc16 = HAL_CRC_Calculate (&hcrc, (uint32_t *) &aqmd2410nsRxData, AQMD2410NS_RX_NUM-3 );
//	}

//	else if ( huart->Instance == huart4.Instance )
//	{
//		if( TJC_RxData[0] == 0xAA && TJC_RxData[1] == 0xBB && TJC_RxData[2] == 0xCC )	//AABBCC是数据包头，校验作用
//		{
//			Set_Freq_Flag = 1;	//设置频率标志位置1，在 Tjshow_Task 任务里处理数据并回显给串口屏
//		}
//		else
//		{
//			memset(TJC_RxData, 0, SIZEOF_TJC_RxData);
//			MX_USART4_UART_Init();
//		}
//
//		HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*) TJC_RxData, SIZEOF_TJC_RxData ) ;
//	}

//	else if ( huart->Instance == huart5.Instance )
//	{
//
//	}
//
//	else if ( huart->Instance == huart6.Instance )
//	{
//
//	}
//
//
//	else if (huart->Instance == huart7.Instance)
//	{
//
//	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
