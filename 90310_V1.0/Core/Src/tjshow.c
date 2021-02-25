/*
 * tjshow.c
 *
 *  Created on: 2020年10月13日
 *      Author: lyh
 */

#ifndef SRC_TJSHOW_C_
#define SRC_TJSHOW_C_

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#include "usart.h"
#include "tjshow.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "user_flash.h"

//uint8_t Set_Freq_Flag = 0 ;
uint8_t TJC_RxData[SIZEOF_TJC_RxData]={0};		//接收串口屏数据的缓存


//在C中，当我们无法列出传递函数的所有实参的类型和数目时,可以用省略号指定参数表
//如：void foo(...); void foo(parm_list,...);
#define  TXBUF_SIZE_MAX  1000
//多个串口printf信息
void USART_printf (UART_HandleTypeDef *huart,char *fmt, ...)
{
    va_list args;
    uint32_t length;
    uint8_t txbuf[TXBUF_SIZE_MAX] = {0};

    va_start(args, fmt);//args存储通过 va_arg 获取额外参数时所必需的信息；fmt最后一个传递给函数的已知的固定参数。
    length = vsnprintf((char *)txbuf, sizeof(txbuf), (char *)fmt, args);
    va_end(args);
    HAL_UART_Transmit(huart, (uint8_t *)txbuf, length, HAL_MAX_DELAY);
    memset(txbuf, 0, TXBUF_SIZE_MAX);
}

//-----------------------------------------------------------------------------
//@name  : void SendToIPC(uint8_t k)
//@brief : 字节发送函数
//@param :
//@retval:
//@date  : 2020/08/03
//@note  :
//-----------------------------------------------------------------------------
void SendToIPC(uint8_t k)
{
	uint8_t i;
    for(i=0;i<3;i++)
    {
		if(k!=0)
		{
		   HAL_UART_Transmit(&huart6,&k,1,10);    //发送一个字节
		   while((__HAL_UART_GET_FLAG(TJHMI_HUART,UART_FLAG_TXE)==RESET)){};   //等待发送结束
		}
    }
}

//-----------------------------------------------------------------------------
//@name  : void HMISend(uint8_t k)
//@brief : 字节发送函数
//@param :
//@retval:
//@date  : 2020/08/03
//@note  : 例子：HMISend(0xFF);//发送 0xff 0xff 0xff
//-----------------------------------------------------------------------------
void HMISend(uint8_t k)
{
	uint8_t i;
    for(i=0;i<3;i++)
    {
		if(k!=0)
		{
		   HAL_UART_Transmit(TJHMI_HUART,&k,1,10);    //发送一个字节
		   while((__HAL_UART_GET_FLAG(TJHMI_HUART,UART_FLAG_TXE)==RESET)){};   //等待发送结束
		}
    }
}

//-----------------------------------------------------------------------------
//@name  : void TJC_Show( float complex_V )
//@brief : 显示合成速度到串口屏
//@param : complex_V：X、Y轴的合成速度
//@retval:
//@date  : 2020/10/13
//@note  :
//-----------------------------------------------------------------------------
void TJC_Show( float complex_V )
{
	int v = 0 ;
	v = complex_V * 100 ;
	USART_printf(TJHMI_HUART,"page0.x0.val=%d",v);
	HMISend(0xFF);
	USART_printf(TJHMI_HUART,"page0.x0.vvs1=2");
	HMISend(0xFF);
}


//-----------------------------------------------------------------------------
//@name  : void TJC_Init( float complex_V )
//@brief : 清除串口屏的显示值
//@param :
//@retval:
//@date  : 2020/11/13
//@note  :
//-----------------------------------------------------------------------------

void TJC_Init(void)
{
	USART_printf(TJHMI_HUART,"page0.x0.val=%d",0);
	HMISend(0xFF);
	USART_printf(TJHMI_HUART,"page1.n1.val=%d",0);
	HMISend(0xFF);
	USART_printf(TJHMI_HUART,"page1.n0.val=%d",0);
	HMISend(0xFF);
}


void FreqHandle(void)
{
	FreqData.data[0] = TJC_RxData[3] ;		//接收串口屏修改的频率值
	FreqData.data[1] = TJC_RxData[4] ;
	FreqData.data[2] = TJC_RxData[5] ;
	FreqData.data[3] = TJC_RxData[6] ;

	HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*)TJC_RxData, SIZEOF_TJC_RxData );
	USART_printf(TJHMI_HUART,"page1.n1.val=%d",FreqData.freq);
//	printf("page1.n1.val=%d",FreqData.freq);	//把接收到的频率回显，让用户确定已成功设置频率
	HMISend(0xFF);


}

//uint32_t SAVEBUF[2]={};
void Tjshow_Task(void *parameter)
{
	HMISend(0xFF);
	memset(TJC_RxData, 0, SIZEOF_TJC_RxData);

//	uint32_t we[2] = {};
//	we[0]=123456;
//    STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)&we,1);
//
//    STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t*)&photo_point, 1);

	while(1)
	{
		HAL_UART_Receive_IT( TJHMI_HUART, (uint8_t*)TJC_RxData, SIZEOF_TJC_RxData );
		TJC_Show( compound_speed2 );

		HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_0);			//提示程序正常运行LED闪烁
		HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);

//		STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t*)GETBUF,1);
//		vTaskDelay(5);
	//	STMFLASH_Read(FLASH_SAVE_ADDR, (uint32_t *)SAVEBUF, 1);
	//	HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_0 );
		vTaskDelay(95);
	}
}

#endif /* SRC_TJSHOW_C_ */
