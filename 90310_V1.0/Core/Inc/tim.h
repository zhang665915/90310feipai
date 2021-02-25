/**
  ******************************************************************************
  * File Name          : TIM.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim12;
/* USER CODE BEGIN Private defines */

typedef enum
{
	Foreward = 0,	//正转
	Reversal ,		//反转
}Direction;



void MX_TIM12_Init(uint16_t arr,uint16_t psc);
void MX_TIM2_Init(uint16_t arr,uint16_t psc);
void MX_TIM8_Init(uint16_t arr,uint16_t psc);
void MX_TIM15_Init(uint16_t arr,uint16_t psc);
void Encoder1_Init(uint16_t psc, uint8_t filter);
void Encoder3_Init(uint16_t psc, uint8_t filter);
void Encoder4_Init(uint16_t psc, uint8_t filter);
void Encoder5_Init(uint16_t psc, uint8_t filter);

void Encoder_Start( TIM_HandleTypeDef *htim );
void Encoder_Stop( TIM_HandleTypeDef *htim );
void Encoder_Counter_Clear( TIM_HandleTypeDef *htim );
Direction Encoder_Get_Direction( TIM_HandleTypeDef *htim );
uint32_t Encoder_Get_Counter( TIM_HandleTypeDef *htim );
void TIM_SetTIMCompare ( TIM_HandleTypeDef* tim_pwmHandle, uint16_t compare );
void TIM_SetTIMfrequency ( TIM_HandleTypeDef* tim_pwmHandle, float speed);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
