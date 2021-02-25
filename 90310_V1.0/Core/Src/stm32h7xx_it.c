/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32h7xx_it.c
 * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include <IO_Config.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx.h"
#include "stm32h7xx_it.h"
#include  "stdlib.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "usart.h"
#include "tim.h"
#include "encoder.h"
#include "dwt_stm32_delay.h"
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;


//uint32_t   ponint_time;//时间轴
//
//uint32_t   ponint_save[10]={0};//各段时间轴
//
//uint32_t   point_flag[10]={0};//各段标志位


//int32_t  a=0;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
	/* USER CODE BEGIN NonMaskableInt_IRQn 0 */

	/* USER CODE END NonMaskableInt_IRQn 0 */
	/* USER CODE BEGIN NonMaskableInt_IRQn 1 */

	/* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void)
{
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void)
{
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void)
{
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void)
{
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1)
	{
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void)
{
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}


/**
* @brief This function handles System tick timer.
*/
extern void xPortSysTickHandler(void);
//systick中断服务函数
int timeCount;
void SysTick_Handler(void)
{
  uint32_t ulReturn;

  timeCount++;
  /* 进入临界段，临界段可以嵌套 */
  ulReturn = taskENTER_CRITICAL_FROM_ISR();

//同步时间计数器、心跳。测试：每1万秒，大约慢0.15秒（CAN在1Mbps速率下、clock的PLL配置为160、5、2、4）
//(为避免数据溢出，建议在工步空闲时段，重置 gHeartbeatCount 变量)

  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif  /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
  }
#endif  /* INCLUDE_xTaskGetSchedulerState */

  /* 退出临界段 */
  taskEXIT_CRITICAL_FROM_ISR( ulReturn );
}


/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB On The Go FS global interrupt.
 */

/* USER CODE BEGIN 1 */
/*  通过定时器2触发中断不断读取编码器的数值 ，定时器2 5us读取一次数据*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
	HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	  /* X1，X2，Y1，Y2代表四轴，其实项目中只用到上下两轴 */
	encoderCount_X1= (accumulation2 * 65536 + ( Encoder_Get_Counter( &htim1))-200);//编码器4计数值，16位定时器
	//encoderCount_X1计数总值，accumulation2溢出计数，每溢出一次中断一次，相当于中断次数计数;
	//Encoder_Get_Counter( &htim1)获取当前定时器的计数值,200为定时器的初值。
	drec_X1 = Encoder_Get_Direction( &htim1 );//获取定时器1的当前方向，实际没用到。

	encoderCount_Y1= (accumulation * 65536 + (Encoder_Get_Counter( &htim3))-2000);//编码器1计数值，16位定时器
	drec_Y1 = Encoder_Get_Direction( &htim3 );

	encoderCount_X2= (accumulation1 * 65536 + (Encoder_Get_Counter( &htim4 ))-200);//编码器2计数值，16位定时器
	drec_X2 = Encoder_Get_Direction( &htim4 );

	encoderCount_Y2 = (Encoder_Get_Counter( &htim5 ));//编码器3计数值，32位定时器
	drec_Y2 = Encoder_Get_Direction(&htim5);

	if(encoderCount_Y1<=A_photo_point)//A_photo_point为上边轴拍照点设定值，负向计数
	{
         if(flag_ab[0]==0)//拍照位置标志位
         {
			GETBUF[0]=encoderCount_Y1;//GETBUF[0]拍照锁存值
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);//驱动拍照
		//	 ponint_save[3] = ponint_time;
			flag_ab[0]++;//完成拍照，置1
         }
         if(point_flag[3]==0)//拍照时间锁存标志
          {
           ponint_save[3] = ponint_time;//ponint_time上边轴拍照时间节点
           point_flag[3]++;//完成时间锁存记录，置1

          }

	}
	else
	{

		   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
		  // flag_ab[0]=0;
	}

  if(encoderCount_Y1>=-500)//回到初始位置，标志位清零
  {
	  flag_ab[0]=0;
  }



	if(encoderCount_Y2<=B_photo_point)//下边轴比上边轴先跑
	{

          if(flag_ab[1]==0)
          {
		   GETBUF[1]=encoderCount_Y2;
		   HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	//	   ponint_save[0] = ponint_time;
		   flag_ab[1]++;
          }
          if(point_flag[0]==0)
          {
           ponint_save[0] = ponint_time;
           point_flag[0]++;

           }

	}
	else
	  {

       HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
      }

	 if(encoderCount_Y2>=-500)
	  {
		  flag_ab[1]=0;
	  }
  /* USER CODE END TIM2_IRQn 1 */
}
//void TIM1_IRQHandler(void)

/* 定时器1的中断 */
void TIM1_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_IRQn 1 */

	  if(__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_UPDATE) != RESET)//中断标志位
     {
		  __HAL_TIM_CLEAR_FLAG(&htim1 , TIM_FLAG_UPDATE);//去中断，避免重复中断

            if((TIM1->CR1&0x10) == 0x10)//向下计数,负向计数，0x为16进制
              {

            	accumulation2--;
              }

            else if((TIM1->CR1&0x10) == 0x00)//向上计数，正向计数

              {

                accumulation2++;
              }

     }
	//  __HAL_TIM_CLEAR_FLAG(&htim3 , TIM_FLAG_UPDATE);
  /* USER CODE END TIM2_IRQn 1 */
}

/* 定时器3的中断 */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
	HAL_TIM_IRQHandler(&htim3);//溢出产生中断

  /* USER CODE BEGIN TIM3_IRQn 1 */
	if(__HAL_TIM_GET_FLAG(&htim3,TIM_FLAG_UPDATE) != RESET)
	{
	  __HAL_TIM_CLEAR_FLAG(&htim3 , TIM_FLAG_UPDATE);

	            if((TIM3->CR1&0x10) == 0x10)//向下计数
	            {

	        	   accumulation--;

	  			}

	            else if((TIM3->CR1&0x10) == 0x00)//向上计数
	  			{

	  				accumulation++;
	  			}

	}

  /* USER CODE END TIM2_IRQn 1 */
}


void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
	HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

	if(__HAL_TIM_GET_FLAG(&htim4,TIM_FLAG_UPDATE) != RESET)
	{

	 __HAL_TIM_CLEAR_FLAG(&htim4 , TIM_FLAG_UPDATE);


			if((TIM4->CR1&0x10) == 0x10)//向下计数
			{

				accumulation1--;

			}

			else if((TIM4->CR1&0x10) == 0x00)//向上计数
			{
				accumulation1++;
			}

	}

  /* USER CODE END TIM4_IRQn 1 */
}
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

//void USART1_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart1);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}


/**
  * @brief This function handles USART2 global interrupt.
  */
//void USART2_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart2);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}


//void USART3_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART3_IRQn 0 */
//
//  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart3);
//  /* USER CODE BEGIN USART3_IRQn 1 */
//
//  /* USER CODE END USART3_IRQn 1 */
//}

//void UART4_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART4_IRQn 0 */
//
//  /* USER CODE END USART4_IRQn 0 */
//  HAL_UART_IRQHandler(&huart4);
//  /* USER CODE BEGIN USART4_IRQn 1 */
//
//  /* USER CODE END USART4_IRQn 1 */
//}

//void UART5_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART3_IRQn 0 */
//
//  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart5);
//  /* USER CODE BEGIN USART3_IRQn 1 */
//
//  /* USER CODE END USART3_IRQn 1 */
//}


//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}

//void UART7_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart7);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}


//void UART8_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART2_IRQn 0 */
//
//  /* USER CODE END USART2_IRQn 0 */
//  HAL_UART_IRQHandler(&huart8);
//  /* USER CODE BEGIN USART2_IRQn 1 */
//
//  /* USER CODE END USART2_IRQn 1 */
//}

//void EXTI0_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI0_IRQn 0 */
//
//  /* USER CODE END EXTI0_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
//  /* USER CODE BEGIN EXTI0_IRQn 1 */
//
//  /* USER CODE END EXTI0_IRQn 1 */
//}

//void EXTI1_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI1_IRQn 0 */
//	//确保是否产生了EXTI Line1中断
//	if(__HAL_GPIO_EXTI_GET_IT(RCollision_Switch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(RCollision_Switch_GPIO_PIN);
//	}
//  /* USER CODE END EXTI1_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
//  /* USER CODE BEGIN EXTI1_IRQn 1 */
//
//  /* USER CODE END EXTI1_IRQn 1 */
//}

//void EXTI2_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI2_IRQn 0 */
//
//	//确保是否产生了EXTI Line2中断
//	if(__HAL_GPIO_EXTI_GET_IT(FRONT_OptoSwitch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(FRONT_OptoSwitch_GPIO_PIN);
//	}
//
//  /* USER CODE END EXTI2_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
//  /* USER CODE BEGIN EXTI2_IRQn 1 */
//
//  /* USER CODE END EXTI2_IRQn 1 */
//}

//void EXTI3_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI3_IRQn 0 */
//
//	//确保是否产生了EXTI Line3中断
//	if(__HAL_GPIO_EXTI_GET_IT(LEFT_OptoSwitch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(LEFT_OptoSwitch_GPIO_PIN);
//	}

//  /* USER CODE END EXTI3_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
//  /* USER CODE BEGIN EXTI3_IRQn 1 */
//
//  /* USER CODE END EXTI3_IRQn 1 */
//}

//void EXTI4_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI4_IRQn 0 */
//
//	//确保是否产生了EXTI Line4中断
//	if(__HAL_GPIO_EXTI_GET_IT(RIGHT_OptoSwitch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(RIGHT_OptoSwitch_GPIO_PIN);
//	}
//  /* USER CODE END EXTI4_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
//  /* USER CODE BEGIN EXTI4_IRQn 1 */
//
//  /* USER CODE END EXTI4_IRQn 1 */
//}

//void EXTI9_5_IRQHandler(void)
//{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

	//确保是否产生了EXTI Line5中断
//	if(__HAL_GPIO_EXTI_GET_IT(FRONT_RIGHT_GPIO_PIN) != RESET)
//	{
//
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(FRONT_RIGHT_GPIO_PIN);
//	}
//
//	//确保是否产生了EXTI Line6中断
//	if(__HAL_GPIO_EXTI_GET_IT(FRONT_MID_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(FRONT_MID_GPIO_PIN);
//	}

	//确保是否产生了EXTI Line7中断
//	if(__HAL_GPIO_EXTI_GET_IT(DS_Limit_UP_GPIO_PIN) != RESET)
//	{
////		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//		SwitchStatus.DS_Limit_UP = TOUCH;
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(DS_Limit_UP_GPIO_PIN);
//	}

	//确保是否产生了EXTI Line8中断
//	if(__HAL_GPIO_EXTI_GET_IT(DS_Limit_DOWN_GPIO_PIN) != RESET)
//	{
////		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//		SwitchStatus.DS_Limit_DOWN = TOUCH;
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(DS_Limit_DOWN_GPIO_PIN);
//	}

	//确保是否产生了EXTI Line9中断，利用定时器消抖
//	if(__HAL_GPIO_EXTI_GET_IT(BACK_MID_GPIO_PIN) != RESET)
//	{
//
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(BACK_MID_GPIO_PIN);
//	}

  /* USER CODE END EXTI9_5_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
//}


//void EXTI15_10_IRQHandler(void)
//{
//  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
//
//	//确保是否产生了EXTI Line11中断
//	if(__HAL_GPIO_EXTI_GET_IT(Front_Safe_Switch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(Front_Safe_Switch_GPIO_PIN);
//	}
//
//	//确保是否产生了EXTI Line12中断
//	if(__HAL_GPIO_EXTI_GET_IT(Back_Safe_Switch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(Back_Safe_Switch_GPIO_PIN);
//	}
//
//	//确保是否产生了EXTI Line13中断
//	if(__HAL_GPIO_EXTI_GET_IT(Left_Safe_Switch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(Left_Safe_Switch_GPIO_PIN);
//	}
//
//	//确保是否产生了EXTI Line14中断
//	if(__HAL_GPIO_EXTI_GET_IT(Right_Safe_Switch_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(Right_Safe_Switch_GPIO_PIN);
//	}
//
//	if(__HAL_GPIO_EXTI_GET_IT(KEY_TEST_GPIO_PIN) != RESET)
//	{
//		HAL_TIM_Base_Start_IT(&htim2);		//启动定时器计时，做20ms的消抖
//
//		//清除中断标志位
//		__HAL_GPIO_EXTI_CLEAR_IT(KEY_TEST_GPIO_PIN);
//	}
//
//  /* USER CODE END EXTI15_10_IRQn 0 */
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
//
//  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);	//Debug按键
//  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
//
//  /* USER CODE END EXTI15_10_IRQn 1 */
//}


//定时器12中断服务函数调用，，周期运行回调，配置定时进入中断
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM12)
	{
      ponint_time++;//计时，1ms进入一次
//      if(flag_ab[1]==1)
//     {
//            if(point_flag[0]==0)
//            {
//    	    ponint_save[0] = ponint_time;
//    	    point_flag[0]++;
//
//            }
//
//
//      }



	}


}

/* 外部中断，记录飞拍各个时间点，用于ponint_time的数值获取 */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
	a++;
	if((point_flag[0]==0)&&(a==1))
	{
		HAL_TIM_Base_Start_IT(&htim12);//开启定时器，，，plc给脉冲开启定时器
	}

	if((a==2)&&(point_flag[1]==0))
	{

		ponint_save[1]=ponint_time;
		point_flag[1]++;
	   //HAL_TIM_Base_Stop(&htim12);
	}
	if((a==3)&&(point_flag[2]==0))
	{

		ponint_save[2]=ponint_time;
		point_flag[2]++;
		// HAL_TIM_Base_Stop(&htim12);
	}
	if((a==4)&&(point_flag[4]==0))
	{

		ponint_save[4]=ponint_time;
		point_flag[4]++;

	}
	if((a==5)&&(point_flag[5]==0))
	{

		ponint_save[5]=ponint_time;
		point_flag[5]++;
	//			HAL_TIM_Base_Stop(&htim12);
	}
	if((a==6)&&(point_flag[6]==0))
	{

		ponint_save[6]=ponint_time;
		point_flag[6]++;
		HAL_TIM_Base_Stop(&htim12);//关闭定时器
	}
//	if((a==7)&&(point_flag[7]==0))
//	{
//
//		ponint_save[7]=ponint_time;
//		point_flag[7]++;
//		HAL_TIM_Base_Stop(&htim12);
//	}
//	if((a==8)&&(point_flag[8]==0))
//	{
//
//		ponint_save[8]=ponint_time;
//		point_flag[8]++;
//
//	}
//	if((a==9)&&(point_flag[9]==0))
//	{
//
//		ponint_save[9]=ponint_time;
//		point_flag[9]++;
//		HAL_TIM_Base_Stop(&htim12);
//	}
  /* USER CODE END EXTI9_5_IRQn 1 */
}
//{
//	if (htim->Instance == TIM2)
	//{
//		HAL_TIM_Base_Stop_IT(&htim2);       //    关闭tim2 及清除中断 TODO:000
//
//
//		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin( FRONT_OptoSwitch_GPIO_PORT, FRONT_OptoSwitch_GPIO_PIN ) )    //前舵轮左限位，再次判断管脚的电平
//		{
//			SwitchStatus.FRONT_OptoSwitch = TOUCH;
////			printf("DEBUG:前防跌落传感器触发！\n");
//		}
//
//		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(LEFT_OptoSwitch_GPIO_PORT,LEFT_OptoSwitch_GPIO_PIN) )    //前舵轮右限位，再次判断管脚的电平
//		{
//			SwitchStatus.LEFT_OptoSwitch = TOUCH;
////			printf("DEBUG:左防跌落传感器触发！\n");
//		}
//
//		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(RIGHT_OptoSwitch_GPIO_PORT,RIGHT_OptoSwitch_GPIO_PIN) )    //前舵轮中限位，再次判断管脚的电平
//		{
//			SwitchStatus.RIGHT_OptoSwitch = TOUCH;
////			printf("DEBUG:右防跌落传感器触发！\n");
//		}
//
//		if (GPIO_PIN_SET == HAL_GPIO_ReadPin( ZY_Stop_Switch_GPIO_PORT,ZY_Stop_Switch_GPIO_PIN ) )    //急停，再次判断管脚的电平
//		{
//			SwitchStatus.ZY_Stop_Switch = TOUCH;
////			printf("DEBUG:急停触发！\n");
//		}
//
//		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin( RCollision_Switch_GPIO_PORT, RCollision_Switch_GPIO_PIN ) )    //前舵轮左限位，再次判断管脚的电平
//		{
//			SwitchStatus.Collision = TOUCH;
////			printf("DEBUG:防撞条传感器触发！\n");
//		}
//
////		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(DS_Limit_UP_GPIO_PORT,DS_Limit_UP_GPIO_PIN) )    //上限位，再次判断管脚的电平
////		{
////			SwitchStatus.DS_Limit_UP = TOUCH;
//////			printf("DEBUG:推杆电机上限位触发！\n");
////		}
////
////		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(DS_Limit_DOWN_GPIO_PORT,DS_Limit_DOWN_GPIO_PIN) )    //下限位，再次判断管脚的电平
////		{
////			SwitchStatus.DS_Limit_DOWN = TOUCH;
//////			printf("DEBUG:推杆电机下限位触发！\n");
////		}
//
//	}
//
//	if ( htim->Instance == TIM3 )
	//{


	//}

//}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
