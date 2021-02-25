/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdio.h"


#define USART1_485_EN(a)	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, a) 		//串口1的485使能脚
#define USART3_485_EN(a)	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, a) 		//串口3的485使能脚

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;		//串口屏
extern UART_HandleTypeDef huart2;		//
extern UART_HandleTypeDef huart3;		//
extern UART_HandleTypeDef huart4;		//
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;		//暂时测试打印
extern UART_HandleTypeDef huart7;		//
extern UART_HandleTypeDef huart8;

#define BOUND_9600		9600
#define BOUND_115200	115200

/*USART1*/
#define USART1_TX_PIN   	GPIO_PIN_9
#define USART1_RX_PIN   	GPIO_PIN_10
#define USART1_GPIO_PORT  	GPIOA
#define __USART1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE();

#define USART1_RS485_EN_PIN		GPIO_PIN_14
#define USART1_RS485_EN_GPIO_PORT  	GPIOB

/*USART2*/
#define USART2_TX_PIN   	GPIO_PIN_5
#define USART2_RX_PIN   	GPIO_PIN_6
#define USART2_GPIO_PORT  	GPIOD
#define __USART2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE();

#define USART2_RS485_EN_PIN		GPIO_PIN_4
#define USART2_RS485_EN_GPIO_PORT  	GPIOD

/*USART3*/
#define USART3_TX_PIN   	GPIO_PIN_8
#define USART3_RX_PIN   	GPIO_PIN_9
#define USART3_GPIO_PORT  	GPIOD
#define __USART3_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE();

#define USART3_RS485_EN_PIN		GPIO_PIN_14
#define USART3_RS485_EN_GPIO_PORT  	GPIOB

/*USART4*/
#define USART4_TX_PIN   	GPIO_PIN_10
#define USART4_RX_PIN   	GPIO_PIN_11
#define USART4_GPIO_PORT  	GPIOC
#define __USART4_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE();

/*USART6*/
#define USART6_TX_PIN   	GPIO_PIN_9
#define USART6_RX_PIN   	GPIO_PIN_14
#define USART6_GPIO_PORT  	GPIOG
#define __USART6_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOG_CLK_ENABLE();

/*USART7*/
#define USART7_TX_PIN   	GPIO_PIN_7
#define USART7_RX_PIN   	GPIO_PIN_8
#define USART7_GPIO_PORT  	GPIOE
#define __USART7_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE();

/*USART8*/
#define USART8_TX_PIN   	GPIO_PIN_0
#define USART8_RX_PIN   	GPIO_PIN_1
#define USART8_GPIO_PORT  	GPIOE
#define __USART8_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE();

#define USART8_RS485_EN_PIN		GPIO_PIN_9
#define USART8_RS485_EN_GPIO_PORT  	GPIOB



#define TJHMI_HUART &huart1		//串口屏通信

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_USART1_UART_Init(uint32_t bound);
void MX_USART2_UART_Init(uint32_t bound);
void MX_USART3_UART_Init(uint32_t bound);
void MX_USART4_UART_Init(uint32_t bound);
void MX_USART5_UART_Init(uint32_t bound);
void MX_USART6_UART_Init(uint32_t bound);
void MX_USART7_UART_Init(uint32_t bound);
void MX_USART8_UART_Init(uint32_t bound);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
