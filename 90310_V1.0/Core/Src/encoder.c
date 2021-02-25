/*
 * encoder.c
 *
 *  Created on: 2020年11月9日
 *      Author: lyh
 */

#include "stm32h7xx_it.h"
#include "encoder.h"
#include "tim.h"
#include "tjshow.h"
#include "IO_Config.h"
#include "stdio.h"
#include <stdlib.h>
#include "user_flash.h"
#include "dwt_stm32_delay.h"
int32_t A_photo_point=-140000;   //上边轴拍照初始值
int32_t B_photo_point=-500000;   //下边轴拍照初始值
uint32_t   ponint_time=0;//时间轴
uint32_t   ponint_save[10]={0};//各段时间轴
uint32_t   point_flag[10]={0};//各段标志位
uint32_t   a=0;//外部PLC脉冲触发中断次数
uint32_t flag_ab[2]={0,0};//a轴和b轴的拍照标志位
uint32_t encoderCount_Y1_1 = 0;
uint32_t encoderCount_X1 = 0, encoderCount_old_X1 = 0 ;	//记录获取的脉冲值
int32_t encoderCount_Y1 = 0, encoderCount_old_Y1 = 0 ;	//记录获取的脉冲值
uint32_t encoderCount_X2 = 0, encoderCount_old_X2 = 0 ;	//记录获取的脉冲值
int32_t encoderCount_Y2 = 0, encoderCount_old_Y2 = 0 ;	//记录获取的脉冲值
uint32_t derta_Count_X1 = 0, derta_Count_old_X1 = 0;	//记录前后两次脉冲值的差
uint32_t derta_Count_Y1 = 0, derta_Count_old_Y1 = 0;	//记录前后两次脉冲值的差
uint32_t derta_Count_X2 = 0, derta_Count_old_X2 = 0;	//记录前后两次脉冲值的差
uint32_t derta_Count_Y2 = 0, derta_Count_old_Y2 = 0;	//记录前后两次脉冲值的差
uint8_t drec_X1 = 0,drec_Y1 = 0, drec_X2 = 0, drec_Y2 = 0;		//电机转动方向
float speed_X1 = 0, speed_Y1 = 0, speed_X2 = 0, speed_Y2 = 0 ;	//各轴的速度
float compound_speed1 = 0, compound_speed2 = 0;	//合成速度

uint32_t arrtest1 = 0,arrtest2=0;

int32_t  accumulation=0;
int32_t  accumulation1=0;
int32_t  accumulation2=0;
uint32_t GETBUF[2]={0,0};//a轴和b轴拍照锁存值
uint32_t SAVEBUF[2]={0,0};
//uint8_t Overflow_counter = 0;


void Get_Encoder_Task(void* parameter)
{
	/* 用于保存上次时间，调用后系统自动更新 */
	static portTickType PreviousWakeTime;
	/* 设置延时时间，将时间转为节拍数 */
	const portTickType TimeIncrement = pdMS_TO_TICKS(derta_Time);	//derta_Time ms

	/* 获取当前系统时间 */
	PreviousWakeTime = xTaskGetTickCount();


	while(1)
	{
		/* -------- X1轴速度计算 ， Encoder4输入 ----------------- */
		//encoderCount_X1 = (Encoder_Get_Counter( &htim1 ));	//获取脉冲值
//		drec_X1 = Encoder_Get_Direction( &htim1 );			//获取定时器计数方向
//		derta_Count_X1 = abs(encoderCount_old_X1 - encoderCount_X1);	//前一次脉冲值-当前获取的脉冲值
//
//		if(derta_Count_X1 > 30000)	//剔除计数溢出的差值
//		{
//			derta_Count_X1 = derta_Count_old_X1;
//		}

//		speed_X1 = ((float)derta_Count_X1 * ONEPULSEOFDIS) / derta_Time;


		/* -------- Y1轴速度计算  Encoder1输入 ----------------- */
//		encoderCount_Y1 = (Encoder_Get_Counter( &htim3 ));	//获取脉冲值
	//	encoderCount_Y1_1= (accumulation * 65536 + (Encoder_Get_Counter( &htim3 )));
//		drec_Y1 = Encoder_Get_Direction( &htim3 );			//获取定时器计数方向
//		derta_Count_Y1 = abs(encoderCount_old_Y1 - encoderCount_Y1);	//前一次脉冲值-当前获取的脉冲值

//		if(derta_Count_Y1 > 30000)	//剔除计数溢出的差值
//		{
//			derta_Count_Y1 = derta_Count_old_Y1;
//		}
//
//		speed_Y1 = ((float)derta_Count_Y1 * ONEPULSEOFDIS) / derta_Time;


		/* ----------- 合成速度1计算  ------------- */

//		compound_speed1 = sqrtf( speed_X1*speed_X1 + speed_Y1*speed_Y1);
//
//		derta_Count_old_X1 = derta_Count_X1;
//		encoderCount_old_X1 = encoderCount_X1;
//		derta_Count_old_Y1 = derta_Count_Y1;
//		encoderCount_old_Y1 = encoderCount_Y1;

		/* -------- X2轴速度计算  Encoder2输入 ----------------- */
	//	encoderCount_X2 = (Encoder_Get_Counter( &htim4 ));	//获取脉冲值
		//drec_X2 = Encoder_Get_Direction( &htim4 );			//获取定时器计数方向
//		derta_Count_X2 = abs(encoderCount_old_X2 - encoderCount_X2);	//前一次脉冲值-当前获取的脉冲值

//		if(derta_Count_X2 > 30000)	//剔除计数溢出的差值
//		{
//			derta_Count_X2 = derta_Count_old_X2;
//		}
//
//		speed_X2 = ((float)derta_Count_X2 * ONEPULSEOFDIS) / derta_Time;


		/* -------- Y2轴速度计算  Encoder3输入 ----------------- */

	//	encoderCount_Y2 = (Encoder_Get_Counter( &htim5 ));	//获取脉冲值
//		drec_Y2 = Encoder_Get_Direction(&htim5);		    //获取定时器计数方向

//		if((encoderCount_Y2>=2197528)&&(encoderCount_Y2<=2800000))
//		{
//
//			 if((flag==0))
//			 {
//				 GETBUF[0]=encoderCount_Y2;
//			//	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//
//				//STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
//				 flag++;
//			 }
//			 else if((flag==1))
//						 {
//							 GETBUF[1]=encoderCount_Y2;
//						//	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//
//							//STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
//							 flag++;
//						 }
//			 else if((flag==2))
//			 						 {
//			 							 GETBUF[2]=encoderCount_Y2;
//			 						//	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//
//			 							//STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
//			 							 flag++;
//			 						 }
//			 else if((flag==3))
//			 						 {
//			 							 GETBUF[3]=encoderCount_Y2;
//			 						//	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//
//			 							//STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
//			 							 flag++;
//			 						 }
//			 else if((flag==4))
//			 						 {
//			 							 GETBUF[4]=encoderCount_Y2;
//			 						//	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//
//			 							//STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
//			 							 flag++;
//			 						 }




		//	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1
		//		                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_SET);
		//}
//		else
//		{
//
//			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1
//							                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
//			flag=0;
//		}


//		derta_Count_Y2 = abs(encoderCount_old_Y2 - encoderCount_Y2);	//前一次脉冲值-当前获取的脉冲值

//		if(derta_Count_Y2 > 30000)	//剔除计数溢出的差值
//		{
//			derta_Count_Y2 = derta_Count_old_Y2;
//		}
//
//		speed_Y2 = ((float)derta_Count_Y2 * ONEPULSEOFDIS) / derta_Time;


		/* ----------- 合成速度2计算  ------------- */

//		compound_speed2 = sqrtf( speed_X2*speed_X2 + speed_Y2*speed_Y2);
//
//		derta_Count_old_X2 = derta_Count_X2;
//		encoderCount_old_X2 = encoderCount_X2;
//		derta_Count_old_Y2 = derta_Count_Y2;
//		encoderCount_old_Y2 = encoderCount_Y2;



		//HAL_GPIO_TogglePin(MCU_OUTPUT1_GPIO_PORT, MCU_OUTPUT1_GPIO_PIN);
//		DWT_Delay_us(5);

		/* 调用绝对延时函数,任务时间间隔为 2 个 tick */
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
	}
}



void Produce_PWM_Task(void* parameter)
{
	/* 用于保存上次时间，调用后系统自动更新 */
	static portTickType PreviousWakeTime;
	/* 设置延时时间，将时间转为节拍数 */
	const portTickType TimeIncrement = pdMS_TO_TICKS(1);	// 1ms

	/* 获取当前系统时间 */
	PreviousWakeTime = xTaskGetTickCount();

//	HAL_TIM_PWM_Start ( &htim8, TIM_CHANNEL_1 );//开启PWM通道1
//	HAL_TIM_PWM_Start ( &htim15, TIM_CHANNEL_1 );//开启PWM通道1

	while(1)
	{
//		if( GPIO_PIN_RESET == HAL_GPIO_ReadPin( FRONT_OptoSwitch_GPIO_PORT, FRONT_OptoSwitch_GPIO_PIN ))
//		{
//			TIM_SetTIMfrequency ( &htim8, compound_speed2 );
//		}


//		if( compound_speed2 <= 0.000001)
//		{
//			HAL_TIM_PWM_Stop ( &htim8, TIM_CHANNEL_1 );//关闭PWM通道1
//			vtest = 5;
//			compound_speed2 = 5;
//		}
//		else
//		{
//			arrtest1 = (uint32_t)64000000/(compound_speed2*FreqData.freq);  //FreqData.freq

//			arrtest1 = (uint32_t)(500000*FreqData.freq)/(1000*compound_speed2);	//500k是定时器分频后的时钟周期，1000是将mm/s转为um/s
//			TIM8 ->ARR = (arrtest1-1);			//通过寄存器修改PWM频率
//			TIM8->CCR1 = ( arrtest1 -1 ) >> 1;	//修改占空比


//			HAL_TIM_PWM_Start ( &htim8, TIM_CHANNEL_1 );//开启PWM通道1

//
//	     if(HAL_GPIO_ReadPin(IN_1_GPIO_Port,IN_1_Pin)==GPIO_PIN_RESET)
//	     {
//	    	 HAL_TIM_Base_Start_IT(&htim12);
//
//
//	     }


//		}

		/* 调用绝对延时函数,任务时间间隔为 5 个 tick */
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
	}
}


void PWM_Start_Task(void* parameter)
{
//	EventBits_t Rx_event; /* 定义一个事件接收变量 */
//
//	uint8_t PWM_Start_flag = 0;
//	uint8_t PWM_Stop_flag = 0;

 while(1)
 {




		HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_0 );


		vTaskDelay(10);
 }

}

