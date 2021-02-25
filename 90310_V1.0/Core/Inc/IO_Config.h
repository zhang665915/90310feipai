//#ifndef __EXTI_H
//#define	__EXTI_H

#ifndef __IO_Config_H
#define __IO_Config_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"



//差分转单端信号芯片使能引脚
/*******************************************************/
#define ST26C32_EN1_GPIO_PORT              GPIOH				//EN1
#define ST26C32_EN1_GPIO_CLK_ENABLE()      __GPIOH_CLK_ENABLE();
#define ST26C32_EN1_GPIO_PIN               GPIO_PIN_13


#define ST26C32_EN2_GPIO_PORT             GPIOH					//EN2
#define ST26C32_EN2_GPIO_CLK_ENABLE()     __GPIOH_CLK_ENABLE();
#define ST26C32_EN2_GPIO_PIN              GPIO_PIN_14

#define ST26C32_EN1(a)	 HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, a) 		//串口1的485使能脚
#define ST26C32_EN2(a)	 HAL_GPIO_WritePin(GPIOH, GPIO_PIN_14, a) 		//串口3的485使能脚


#define MCU_OUTPUT1_GPIO_PORT             GPIOD					//MCU_OUTPUT1
#define MCU_OUTPUT1_GPIO_CLK_ENABLE()     __GPIOD_CLK_ENABLE();
#define MCU_OUTPUT1_GPIO_PIN              GPIO_PIN_3

//#define Left_Safe_Switch_GPIO_PORT                GPIOF
//#define Left_Safe_Switch_GPIO_CLK_ENABLE()        __GPIOF_CLK_ENABLE();
//#define Left_Safe_Switch_GPIO_PIN                 GPIO_PIN_13
//#define Left_Safe_Switch_EXTI_IRQ                 EXTI15_10_IRQn
//#define Left_Safe_Switch_IRQHandler               EXTI15_10_IRQHandler

//按键
/*******************************************************/
#define KEY_PAUSE_GPIO_PORT				   GPIOA							//野火debug key
#define KEY_PAUSE_GPIO_CLK_ENABLE()        __GPIOA_CLK_ENABLE();
#define KEY_PAUSE_GPIO_PIN                 GPIO_PIN_0
#define KEY_PAUSE_EXTI_IRQ                 EXTI0_IRQn
#define KEY_PAUSE_IRQHandler               EXTI0_IRQHandler

#define KEY_STOP_GPIO_PORT				   GPIOC							//野火debug key
#define KEY_STOP_KEY_CLK_ENABLE()          __GPIOC_CLK_ENABLE();
#define KEY_STOP_GPIO_PIN                  GPIO_PIN_13
#define KEY_STOP_EXTI_IRQ                  EXTI15_10_IRQn
#define KEY_STOP_IRQHandler                EXTI15_10_IRQHandler

#define KEY_TEST_GPIO_PORT			   	   GPIOH							//2321 test key
#define KEY_TEST_GPIO_CLK_ENABLE()     	   __GPIOH_CLK_ENABLE();
#define KEY_TEST_GPIO_PIN             	   GPIO_PIN_15
#define KEY_TEST_EXTI_IRQ                  EXTI15_10_IRQn
#define KEY_TEST_IRQHandler                EXTI15_10_IRQHandler

#define IN_1_Pin                   GPIO_PIN_6
#define IN_1_GPIO_Port             GPIOB
#define IN_2_Pin                   GPIO_PIN_7
#define IN_2_GPIO_Port             GPIOB
#define IN_3_Pin                   GPIO_PIN_8
#define IN_3_GPIO_Port             GPIOB
#define IN_4_Pin                   GPIO_PIN_9
#define IN_4_GPIO_Port             GPIOB


#pragma pack(push)	//保存内存字节对齐
#pragma pack(1)		//改为一个字节对齐




#pragma pack(pop)	//回复内存字节对齐


void ST26C32_EN_Config(void);
//void GPIO_PinReverse(GPIO_TypeDef*GPIOx,uint16_t GPIO_pin);

#ifdef __cplusplus
}
#endif
#endif /* IO_Config_H */
