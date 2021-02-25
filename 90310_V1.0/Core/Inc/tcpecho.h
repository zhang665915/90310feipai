/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#ifndef LWIP_TCPECHO_H
#define LWIP_TCPECHO_H

#include "stdint.h"


#pragma pack(push)
#pragma pack(1)

//TCP控制包包头
#define AGV_TCP_PACKHEADER   (uint16_t)0xBCCD

//本地ID号
#define TCP_LOCAL_ID         (uint8_t)0x01
//本地发送数据长度
#define TCP_LOCAL_Len        (uint8_t)0x04

uint8_t tcpDataArray[32]; //飞拍发送的数据数组

//功能码
 typedef enum
 {
	SET_PHOTO_PIONT1 	= 0x1001,
	SET_PHOTO_PIONT2 	= 0x1002,
//	JOYSTICK_SETTINGS 		= 0x1005,   // 遥控手柄参数设置
//	WHEEL_CALIBRATION   	= 0x1006,   // 轮参数标定

 } AGV_TcpFuntionCode;




// 发送数据共用体
//typedef union
//{
//
//      JoystickSetting       joustick_settings;
//      WheelCalibration      wheel_calibration;
//
//} UnionControlData;

 //接收数据总包
typedef struct
{
    uint16_t pack_head; //包头
    uint16_t pack_length; //数据段长度
    AGV_TcpFuntionCode funtion_code; //功能码2字节
    uint32_t  aaa;    //数据
    uint16_t crc16_t; //CRC校验(功能码+控制数据)
} ControlPack;

//接收字符串
// typedef struct
//   {
//
//
//
//
//   }ControlPack;







#pragma pack(pop)






void tcpecho_init(void);

#endif /* LWIP_TCPECHO_H */
