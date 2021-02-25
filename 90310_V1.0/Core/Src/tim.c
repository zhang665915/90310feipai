/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "tim.h"
#include "tjshow.h"
#include "main.h"
#include "encoder.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;	//Encoder4编码器输入
TIM_HandleTypeDef htim2;	//时间计数，用于求电机转速
TIM_HandleTypeDef htim3;	//Encoder1编码器输入
TIM_HandleTypeDef htim4;	//Encoder2编码器输入
TIM_HandleTypeDef htim5;	//Encoder3编码器输入
TIM_HandleTypeDef htim8;	//PWM输出
TIM_HandleTypeDef htim12;	//时间计数，记录时间节点

/* TIM1 init function */
/*****************************************************************
 * @brief  Encoder1_Init
 *         编码器1初始化配置
 * @param  psc---预分频， filter----内部滤波数 （取值范围 0~15）
 *         默认psc=0, filter=0;
 * @retval
 * @note   预分频是对编码器输入信号（A或B）进行分频, 例如编码器的分辨率是3600 pulses/圈，
 *         设定4分频（psc = 4-1）, 则编码器计数是按 900 pulses/圈，
 *         如一圈计数超出计数器（16bit 或 32bit）, 建议分频或定时清零计数器，防止计数溢出。
 ****************************************************************/
void Encoder1_Init( uint16_t psc, uint8_t filter )
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = psc;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;		//电机分辨率*4  2048  (60000-1)*4
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode =  TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = filter;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = filter;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  TIM1->CNT = 200; //避免电机抖动频繁进入中断
  __HAL_TIM_CLEAR_FLAG(&htim1 , TIM_FLAG_UPDATE);//去除中断标志位
  HAL_TIM_Encoder_Start_IT(&htim1,TIM_CHANNEL_ALL);//编码器模式下开启定时器
}

//通用定时器2中断初始化,定时器2在APB1上，APB1的定时器时钟为240MHz
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器2!(定时器2挂在APB1上，时钟为HCLK/2)
void MX_TIM2_Init(uint16_t arr,uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = psc;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = arr;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_FLAG(&htim2 , TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim2);
}

/* TIM3 init function */
void Encoder3_Init( uint16_t psc, uint8_t filter )
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = psc;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = filter;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = filter;

  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  TIM3->CNT = 2000;//编码器3初始值
  __HAL_TIM_CLEAR_FLAG(&htim3 , TIM_FLAG_UPDATE);//避免一开始初始化进入中断

  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
}
void Encoder4_Init(uint16_t psc, uint8_t filter)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = psc;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = filter;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = filter;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  TIM4->CNT = 200;
    __HAL_TIM_CLEAR_FLAG(&htim4 , TIM_FLAG_UPDATE);
  HAL_TIM_Encoder_Start_IT(&htim4,TIM_CHANNEL_ALL);
}
/* TIM5 init function */
void Encoder5_Init(uint16_t psc, uint8_t filter)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = psc;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = filter;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = filter;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}



void MX_TIM12_Init(uint16_t arr,uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = psc;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = arr;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_TIM_CLEAR_FLAG(&htim12 , TIM_FLAG_UPDATE);
}

//设置TIM通道的占空比
//compare:比较值

void TIM_SetTIMCompare ( TIM_HandleTypeDef* tim_pwmHandle, uint16_t compare )
{
	if(tim_pwmHandle->Instance==TIM8)
	{
		TIM8->CCR1 = compare ;
	}
	else if (tim_pwmHandle->Instance==TIM15)
	{
		TIM15 ->CCR1 = compare ;
	}
}

//设置TIM通道的频率
//arr:重装载值

void TIM_SetTIMfrequency ( TIM_HandleTypeDef* tim_pwmHandle, float speed )
{
	uint32_t arr[2] = {0,0};

	if(tim_pwmHandle->Instance==TIM8)
	{
		arr[0] = (uint32_t)speed;//*FreqData.freq;	//TODO:控制输出频率有待确认11/5
		TIM8 ->ARR = (arr[0]-1);
		TIM8->CCR1 = ( arr[0] -1 ) >> 1;		//注意：不同的通道对应不同的CCR，这里是CH1，所以用 TIM capture/compare register 1

	}
	else if (tim_pwmHandle->Instance==TIM15)
	{
		arr[1] = speed; //FreqData.freq;	//TODO:控制输出频率有待确认11/5
		TIM15 ->ARR = (arr[1]-1);
		TIM15->CCR1 = ( arr[1] -1 ) >> 1;		//注意：不同的通道对应不同的CCR
	}

}

/* TIM8 init function */
//void MX_TIM8_Init(uint16_t arr,uint16_t psc)
//{
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  htim8.Instance = TIM8;
//  htim8.Init.Prescaler = psc;	//定时器分频
//  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;	//向上计数模式
//  htim8.Init.Period = arr;		//自动重装载值
//  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim8.Init.RepetitionCounter = 0;
//  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;	//模式选择PWM1
//  sConfigOC.Pulse = arr/2;				//设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;//TIM_OCPOLARITY_HIGH; //	//输出比较极性为高
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;//TIM_OCNPOLARITY_HIGH;//
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)	//配置TIM8通道1
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;//TIM_BREAKPOLARITY_HIGH; //
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
//  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_LOW;//TIM_BREAK2POLARITY_HIGH; //
//  sBreakDeadTimeConfig.Break2Filter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_TIM_MspPostInit(&htim8);
//
////  HAL_TIM_PWM_Start ( &htim8, TIM_CHANNEL_1 );//开启PWM通道1
//
//}
/* TIM15 init function */
//void MX_TIM15_Init(uint16_t arr,uint16_t psc)
//{
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};
//  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
//
//  htim15.Instance = TIM15;
//  htim15.Init.Prescaler = psc;
//  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim15.Init.Period = arr;
//  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim15.Init.RepetitionCounter = 0;
//  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sConfigOC.OCMode = TIM_OCMODE_PWM1;	//模式选择PWM1
//  sConfigOC.Pulse = arr/2;				//设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
//  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;	//输出比较极性为高
//  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
//  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
//  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
//  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)	//配置TIM15通道1
//  {
//    Error_Handler();
//  }
//  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
//  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
//  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
//  sBreakDeadTimeConfig.DeadTime = 0;
//  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
//  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
//  sBreakDeadTimeConfig.BreakFilter = 0;
//  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
//  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  HAL_TIM_MspPostInit(&htim15);
//
////  HAL_TIM_PWM_Start ( &htim15, TIM_CHANNEL_1 );//开启PWM通道1
//
//}

//void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
//{
//
//  if(tim_pwmHandle->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspInit 0 */
//
//  /* USER CODE END TIM8_MspInit 0 */
//    /* TIM8 clock enable */
//    __HAL_RCC_TIM8_CLK_ENABLE();
//  /* USER CODE BEGIN TIM8_MspInit 1 */
//
//  /* USER CODE END TIM8_MspInit 1 */
//  }
//  else if(tim_pwmHandle->Instance==TIM15)
//  {
//  /* USER CODE BEGIN TIM15_MspInit 0 */
//
//  /* USER CODE END TIM15_MspInit 0 */
//    /* TIM15 clock enable */
//    __HAL_RCC_TIM15_CLK_ENABLE();
//  /* USER CODE BEGIN TIM15_MspInit 1 */
//
//  /* USER CODE END TIM15_MspInit 1 */
//  }
//}
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
//{
//
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(timHandle->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspPostInit 0 */
//
//  /* USER CODE END TIM8_MspPostInit 0 */
////    __HAL_RCC_GPIOC_CLK_ENABLE();
//	  __HAL_RCC_GPIOI_CLK_ENABLE();
//    /**TIM8 GPIO Configuration
//    PC6     ------> TIM8_CH1
//    */
//	GPIO_InitStruct.Pin = GPIO_PIN_5;	//控制板卡使用PI5
////    GPIO_InitStruct.Pin = GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
////    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
//  /* USER CODE BEGIN TIM8_MspPostInit 1 */
////    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_RESET);
//  /* USER CODE END TIM8_MspPostInit 1 */
//  }
//  else if(timHandle->Instance==TIM15)
//  {
//  /* USER CODE BEGIN TIM15_MspPostInit 0 */
//
//  /* USER CODE END TIM15_MspPostInit 0 */
//
//    __HAL_RCC_GPIOE_CLK_ENABLE();
//    /**TIM15 GPIO Configuration
//    PE5     ------> TIM15_CH1
//    */
//    GPIO_InitStruct.Pin = GPIO_PIN_5;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;//GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF4_TIM15;
//    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
//
//  /* USER CODE BEGIN TIM15_MspPostInit 1 */
//
//  /* USER CODE END TIM15_MspPostInit 1 */
//  }
//
//}

//void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
//{
//
//  if(tim_pwmHandle->Instance==TIM8)
//  {
//  /* USER CODE BEGIN TIM8_MspDeInit 0 */
//
//  /* USER CODE END TIM8_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM8_CLK_DISABLE();
//  /* USER CODE BEGIN TIM8_MspDeInit 1 */
//
//  /* USER CODE END TIM8_MspDeInit 1 */
//  }
//  else if(tim_pwmHandle->Instance==TIM15)
//  {
//  /* USER CODE BEGIN TIM15_MspDeInit 0 */
//
//  /* USER CODE END TIM15_MspDeInit 0 */
//    /* Peripheral clock disable */
//    __HAL_RCC_TIM15_CLK_DISABLE();
//  /* USER CODE BEGIN TIM15_MspDeInit 1 */
//
//  /* USER CODE END TIM15_MspDeInit 1 */
//  }
//}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM2)
	  {
		  /* USER CODE BEGIN TIM2_MspInit 0 */

		  /* USER CODE END TIM2_MspInit 0 */
		    /* TIM2 clock enable */
		    __HAL_RCC_TIM2_CLK_ENABLE();


		    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		    HAL_NVIC_EnableIRQ(TIM2_IRQn);
		    /* TIM2 interrupt Init */

		  /* USER CODE BEGIN TIM2_MspInit 1 */

		  /* USER CODE END TIM2_MspInit 1 */
	  }

	  if(tim_baseHandle->Instance==TIM12)
	  {
	  /* USER CODE BEGIN TIM12_MspInit 0 */

	  /* USER CODE END TIM12_MspInit 0 */
	    /* TIM12 clock enable */
	    __HAL_RCC_TIM12_CLK_ENABLE();

	    /* TIM12 interrupt Init */
	    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
	  /* USER CODE BEGIN TIM12_MspInit 1 */

	  /* USER CODE END TIM12_MspInit 1 */
	  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
	  if(tim_baseHandle->Instance==TIM2)
	  {
	  /* USER CODE BEGIN TIM2_MspDeInit 0 */

	  /* USER CODE END TIM2_MspDeInit 0 */
	    /* Peripheral clock disable */
	    __HAL_RCC_TIM2_CLK_DISABLE();

	    /* TIM2 interrupt Deinit */
	    HAL_NVIC_DisableIRQ(TIM2_IRQn);
	    /* TIM4 interrupt Deinit */

	  /* USER CODE BEGIN TIM2_MspDeInit 1 */

	  /* USER CODE END TIM2_MspDeInit 1 */
	  }
	  if(tim_baseHandle->Instance==TIM12)
	    {
	    /* USER CODE BEGIN TIM12_MspDeInit 0 */

	    /* USER CODE END TIM12_MspDeInit 0 */
	      /* Peripheral clock disable */
	      __HAL_RCC_TIM12_CLK_DISABLE();

	      /* TIM12 interrupt Deinit */
	      HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
	    /* USER CODE BEGIN TIM12_MspDeInit 1 */

	    /* USER CODE END TIM12_MspDeInit 1 */
	    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_encoderHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspInit 1 */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USER CODE END TIM1_MspInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
       HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);		//控制板卡使用PC6、PC7
    /* TIM1 interrupt Init */
       HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
       HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM4_MspInit 1 */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE END TIM4_MspInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspInit 0 */

  /* USER CODE END TIM5_MspInit 0 */
    /* TIM5 clock enable */
    __HAL_RCC_TIM5_CLK_ENABLE();

    __HAL_RCC_GPIOH_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();

    /**TIM5 GPIO Configuration
    PA0     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
//    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;//GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspInit 1 */
//    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USER CODE END TIM5_MspInit 1 */
  }



}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{

  if(tim_encoderHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PE9     ------> TIM1_CH1
    PE11     ------> TIM1_CH2
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_9|GPIO_PIN_11);

  /* USER CODE BEGIN TIM1_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
  /* USER CODE END TIM1_MspDeInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6|GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

  /* USER CODE END TIM4_MspDeInit 1 */
  }
  else if(tim_encoderHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM5_CLK_DISABLE();

    /**TIM5 GPIO Configuration
    PA0     ------> TIM5_CH1
    PA1     ------> TIM5_CH2
    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN TIM5_MspDeInit 1 */
//    HAL_NVIC_DisableIRQ(TIM5_IRQn);
  /* USER CODE END TIM5_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/*****************************************************************
 * @brief  Encoder_Start
 *         启动编码器计数
 * @param
 * @retval
 * @note
 ****************************************************************/
void Encoder_Start( TIM_HandleTypeDef *htim )
{
	HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

/*****************************************************************
 * @brief  Encoder_Stop
 *         停止编码器计数
 * @param
 * @retval
 * @note
 ****************************************************************/
void Encoder_Stop( TIM_HandleTypeDef *htim )
{
	HAL_TIM_Encoder_Stop(htim, TIM_CHANNEL_ALL);
}

/*****************************************************************
 * @brief  Encoder1_Counter_Clr
 *         编码器1计数器清零
 * @param
 * @retval
 * @note
 ****************************************************************/
void Encoder_Counter_Clear( TIM_HandleTypeDef *htim )
{
	__HAL_TIM_SET_COUNTER(htim, 0x00000000);
}

/*****************************************************************
 * @brief  Encoder_Get_Direction
 *         获取电机的转向
 * @param
 * @retval Dir = 0 正转，Dir = 1 反转
 * @note
 ****************************************************************/
Direction Encoder_Get_Direction( TIM_HandleTypeDef *htim )
{
	uint8_t Dir = 0 ;
	Dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);
	return Dir ;
}

/*****************************************************************
 * @brief  Encoder_Get_Counter
 *         获取定时器计数器的计数值，即编码器的脉冲数
 * @param
 * @retval
 * @note
 ****************************************************************/

uint32_t Encoder_Get_Counter( TIM_HandleTypeDef *htim )
{
	uint32_t count = 0 ;
	count = __HAL_TIM_GET_COUNTER(htim);
	return count ;
}
/*****************************************************************
 * @brief  Encoder_Get_Counter
 *         设置定时器计数器的计数值，即编码器的脉冲数
 * @param
 * @retval
 * @note
 ****************************************************************/

void Set_Counter( TIM_HandleTypeDef *htim ,uint32_t Set_Counter)
{
	if (htim == &htim3)
	{
		encoderCount_Y1=Set_Counter;

	}
	if (htim == &htim5)
		{
		encoderCount_Y2=Set_Counter;

		}

}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
