/*
 * encoder.h
 *
 *  Created on: 2020年11月9日
 *      Author: lyh
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"


extern uint32_t   ponint_time;//时间轴
extern uint32_t   ponint_save[10];//各段时间轴
extern uint32_t   point_flag[10];//各段标志位
extern uint32_t  a;
extern int32_t  accumulation;
extern int32_t  accumulation1;
extern int32_t  accumulation2;
extern uint32_t encoderCount_X1, encoderCount_old_X1;	//记录获取的脉冲值
extern int32_t encoderCount_Y1, encoderCount_old_Y1;	//记录获取的脉冲值
extern uint32_t encoderCount_X2, encoderCount_old_X2;	//记录获取的脉冲值
extern int32_t encoderCount_Y2, encoderCount_old_Y2;	//记录获取的脉冲值
extern uint32_t derta_Count_X1, derta_Count_old_X1;		//记录前后两次脉冲值的差
extern uint32_t derta_Count_Y1, derta_Count_old_Y1;		//记录前后两次脉冲值的差
extern uint32_t derta_Count_X2, derta_Count_old_X2;		//记录前后两次脉冲值的差
extern uint32_t derta_Count_Y2, derta_Count_old_Y2;		//记录前后两次脉冲值的差
extern uint8_t drec_X1,drec_Y1, drec_X2, drec_Y2;		//电机转动方向
extern float speed_X1, speed_Y1, speed_X2, speed_Y2;	//各轴的速度
extern float compound_speed1, compound_speed2;			//合成速度

extern uint32_t encoderCount_Y1_1 ;
extern uint32_t GETBUF[2];
extern uint32_t flag_ab[2];
extern int32_t A_photo_point;
extern int32_t B_photo_point;

#define ONEPULSEOFDIS  (float)0.05 	//行走0.05um产生一个脉冲
#define derta_Time	   (float)1  	//2ms获取一次脉冲数
#define PWM_START_EVENT   ( 0x01 << 0 )	//启动PWM事件组位
#define RX_SET_FREQ_EVENT ( 0x01 << 1 )	//接收串口屏设置PWM频率事件组位
extern EventGroupHandle_t PWM_Start_Event ; //创建开启PWM事件
extern EventGroupHandle_t Rx_Set_Freq_Event ; //接收串口屏设置频率事件

void Get_Encoder_Task(void *paramter);
void Produce_PWM_Task(void* parameter);
void PWM_Start_Task(void* parameter);

#endif /* INC_ENCODER_H_ */
