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
#include "udpecho.h"
#include "lwip/opt.h"

#if LWIP_NETCONN
#include "lwip/api.h"
#include "lwip/sys.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "usart.h"


#include <IO_Config.h>


/*-----------------------------------------------------------------------------------*/



/* 小端转大端 */
// 16位
void LowToHigh_16(void *data)
{
	uint8_t returnData[2] = {0};
	uint8_t * dataPoint = data;

	for(uint8_t i = 0; i < 2; i++)
	{
		returnData[1-i] = dataPoint[i];
	}
	for(uint8_t i = 0; i < 2; i++)
	{
		dataPoint[i] = returnData[i];
	}
}

// 32位
void LowToHigh_32(void *data)
{
	uint8_t returnData[4] = {0};
	uint8_t * dataPoint = data;
	for(uint8_t i = 0; i < 4; i++)
	{
		returnData[3-i] = dataPoint[i];
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		dataPoint[i] = returnData[i];
	}
}

// 64位
void LowToHigh_64(void *data)
{
	uint8_t returnData[8] = {0};
	uint8_t * dataPoint = data;
	for(uint8_t i = 0; i < 8; i++)
	{
		returnData[7-i] = dataPoint[i];
	}
	for(uint8_t i = 0; i < 8; i++)
	{
		dataPoint[i] = returnData[i];
	}
}




static void udpecho_thread( void *arg )
{
    struct netconn *conn;
    ip4_addr_t ipaddr;  				//用于构造目的IP地址
    err_t err;							//通用错误调试返回值
    LWIP_UNUSED_ARG(arg);
    
    conn = netconn_new(NETCONN_UDP); 	//新建UDP netconn结构体
    if (conn != NULL)
        printf("\n创建UDP连接完成!\n");

    err = netconn_bind( conn, IP_ADDR_ANY, 7762 );  //绑定本地IP和端口
    if (err == ERR_OK)
        printf("UDP本地绑定完成！\n");

//  xSemaphoreTake( SyncCountSem_Handle, portMAX_DELAY );	//线程同步，等待轮毂电机初始化完成

    /*以下代码为构建一个目标IP地址并连接*/
    IP4_ADDR(&ipaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2,  DEST_IP_ADDR3);
    err = netconn_connect(conn, &ipaddr, 7762);
    printf("UDP netconn目标连接结果为：%d\n", err);
    
    /*以下代码为检验UDP是否已获取上位机IP地址*/
	u32_t addr = 0;
	u16_t port = 0 ;
	u8_t remot_addr[4];
    /* 用于校验TCP邮箱消息 */
	err = netconn_getaddr(conn,(ip_addr_t*)(&addr),&port,0);
	remot_addr[3] = (uint8_t)(addr >> 24);
	remot_addr[2] = (uint8_t)(addr>> 16);
	remot_addr[1] = (uint8_t)(addr >> 8);
	remot_addr[0] = (uint8_t)(addr);
	printf("主机%d.%d.%d.%d连接上UDP,主机端口号为:%d\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3],port);


	/* 用于保存上次时间。调用后系统自动更新 */
	static portTickType PreviousWakeTime;
	/* 设置延时时间，将时间转为节拍数 */
	const portTickType TimeIncrement = pdMS_TO_TICKS(20);	//50HZ = 20ms
	/* 获取当前系统时间 */
	PreviousWakeTime = xTaskGetTickCount();

    struct netbuf *buf;   //新建存放数据的netbuf地址指针
    int udpdataLen = 0;		//UDP数据包长度

    while (1)
    {
		/* 调用绝对延时函数,任务时间间隔为 20 个 tick */
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);

		HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_0);								//测试LED
		HAL_GPIO_TogglePin(GPIOI,GPIO_PIN_1);

//		udpdataLen = AGVDataReturn();
		buf = netbuf_new();
		netbuf_alloc( buf, udpdataLen );   		//分配pbuf空间
//		buf->p->payload = agvDataArray;
		err = netconn_send(conn, buf);   		//发送UDP实时数据

		if (err != ERR_OK)
		{
			printf("发送失败:err的值为：%d\n",err);
		}
		else
		{
	//		printf("发送成功:err的值为：%d\n",err);
		}
		netbuf_delete(buf);
    }
}


/*-----------------------------------------------------------------------------------*/
extern TaskHandle_t ROS_UDP_TASK_Handle;	/* ROS_UDP任务句柄 */
void udpecho_init(void)
{
	ROS_UDP_TASK_Handle = sys_thread_new("udpecho_thread", udpecho_thread, NULL, 1024, 7 );
}

#endif /* LWIP_NETCONN */
