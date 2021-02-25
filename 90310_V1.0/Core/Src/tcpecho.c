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
#include "tcpecho.h"
#include "lwip/opt.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#if LWIP_NETCONN
#include "lwip/sys.h"
#include "lwip/api.h"
#include "tcp.h"
#include "string.h"
#include "crc.h"
#include "encoder.h"

#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<string.h>
#include "encoder.h"
#include "tim.h"
//#include "tim.h"
//-----------------------------------------------------------------------------
//@name  : uint8_t DataReturn(uint8_t OP_Code, uint8_t send_data)
//@brief : TCP发送数据处理函数
//@param :
//@retval:
//@date  :
//@note  :
//-----------------------------------------------------------------------------

uint8_t DataReturn(uint16_t OP_Code, uint32_t send_data)
{
	uint8_t num = 0;
	uint16_t crc16 = 0xFFFF;

	memset(tcpDataArray,0,32); //清零

	tcpDataArray[0] = 0xBC;
	tcpDataArray[1] = 0xBD;
	tcpDataArray[2] = TCP_LOCAL_Len;
	tcpDataArray[3] = OP_Code>>8;
	tcpDataArray[4] = OP_Code;
	tcpDataArray[5] = send_data>>24;
	tcpDataArray[6] = send_data>>16;
	tcpDataArray[7] = send_data>>8;
	tcpDataArray[8] = send_data;
	num = 9;

	//CRC校验
	crc16 = HAL_CRC_Calculate ( &hcrc, (uint32_t *)&tcpDataArray[2], num-2 );

	memcpy(tcpDataArray + num, &crc16, 2);
	num += 2;

	return num;
}
/*-----------------------------------------------------------------------------
 * @name  : my_uart_strstr
 * @brief : 一种字符序列查找方法
 * @param : charList 数据区地址
 *          aimCMD 指令头地址
 *          cmdLength 指令长度
 * @retval: 字符序列的地址
 * @date  : 2020/11/20
 * @note  :
 * ---------------------------------------------------------------------------*/

/*--------------------------*/
static const char * my_uart_strstr(const char * charList, const char * aimCMD, int cmdLength)
{
	const char * cmdHeader = charList;
	const char * end = &charList[200];
	while(cmdHeader < end)
	{
		cmdHeader = strchr(cmdHeader, aimCMD[0]);  //查找指令头
		if(cmdHeader > end || cmdHeader == NULL)  //如果不在缓冲区中
			return  NULL;
		if(cmdHeader <= &charList[200 - cmdLength]) //如果剩下缓冲区长度已不足以存放一个指令长度
		{
			if(memcmp(cmdHeader, aimCMD, cmdLength) != 0)
				cmdHeader++;  //如果不匹配则跳过该字节
			else
				return cmdHeader;  //查找成功，返回指令位置
		}
		else
		{
			return NULL;
		}
	}
	return NULL;
}





char point_buf[128] = { 0 };

//定义数组变量
const char setpoint_A[]={'s','e','t','p','o','i','n','t','A'};
const char setpoint_B[]={'s','e','t','p','o','i','n','t','B'};
const char    clear_A[]={'c','l','e','a','r','A'};
const char    clear_B[]={'c','l','e','a','r','B'};
const char getlatch_A[]={'g','e','t','l','a','t','c','h','A'};
const char getlatch_B[]={'g','e','t','l','a','t','c','h','B'};
const char  setdone[]={'s','e','t','d','o','n','e'};
const char  flag_point_A[]={'f','l','a','g','A'};
const char  flag_point_B[]={'f','l','a','g','B'};
const char clear_FLL[]={'c','l','e','a','r','F','L','L'};





static void tcpcmd_thread(void *arg)	//TCP线程：10001，包括上电时候命令和心跳
{
 struct netconn *conn, *newconn ;
 err_t err = 0 ;

 LWIP_UNUSED_ARG(arg);		//消除关于未使用参数的编译器警告
 conn = netconn_new(NETCONN_TCP);	//创建TCP连接结构体
 netconn_bind(conn, IP_ADDR_ANY, 7862);	//绑定本地IP地址和端口号
 LWIP_ERROR("tcpecho: invalid conn",(conn != NULL), return;);
 printf("\nTCP7862：本地绑定完成！\n");


 /* TCP服务器处于监听状态 */
 netconn_listen(conn);
 printf("TCP10001 wait connection\n");

 while(1)
 {
	 /* 等待一个新的客户端前来连接，包括完成三次握手，若无连接则一直阻塞 */
	 err = netconn_accept(conn, &newconn);

	 /* 新的客户端连接成功 */
	 if( err == ERR_OK )
	 {
		 printf("TCP10001 was connected successfully\n");

		 /* 以太网数据接收处理相关变量 */
		 struct netbuf *buf ;
		 void * databuf = NULL ; 	//指针，用于保存接收数据包的pbuf起始地址
		 u16_t len = 0 ;			//用于记录TCP控制包的长度
		// u8_t send_data_len=0;    //发送数据长度
		 const char * cmdHeader = NULL;
		 const char * pos = NULL;
		 const char * end = NULL;

		 /* 用于获取工控机的IP地址 */
		 u32_t addr = 0;
		 u16_t port  = 0;
		 u8_t remot_addr[4];
		 err = netconn_getaddr(newconn , (ip_addr_t*)(&addr), (&port), 0);
		remot_addr[3] = (uint8_t)(addr >> 24);
		remot_addr[2] = (uint8_t)(addr>> 16);
		remot_addr[1] = (uint8_t)(addr >> 8);
		remot_addr[0] = (uint8_t)(addr);
		printf("客服端%d.%d.%d.%d连接上服务器TCP7862,端口号为:%d\r\n",remot_addr[0], remot_addr[1],remot_addr[2],remot_addr[3],port);

		newconn->recv_timeout = 000 ;	//以太网接收不永久阻塞，只阻塞3s

		/*等待数据接收完成*/
		while(netconn_recv(newconn,& buf ) == ERR_OK )
		{
			printf("agv_TCP\n");

			/* 拷贝以太网接收缓冲区的内容*/
			do{ netbuf_data( buf, &databuf, &len ); }
			while( netbuf_next(buf) >= 0 ) ;

			 //   netbuf_data(buf, &databuf, &len);

			/*-----------PLC给A轴设定一个值，经网络通信传输给stm32芯片，A_photo_point为上边轴拍照点设定值---------------*/
 if(len != 0)
 {
// 设置A轴拍照点
	    cmdHeader = my_uart_strstr((const char *)databuf,
	    		                              setpoint_A,
								       sizeof(setpoint_A));//
	    end = strchr(cmdHeader, '\n');//判断是否有结束符，如有，则返回1，strchr函数功能为在一个串中查找给定字符的第一个匹配之处，
	                                  //即在参数 str 所指向的字符串中搜索第一次出现字符 c（一个无符号字符）的位置
  if(cmdHeader != NULL && end != NULL)
  {
	pos = cmdHeader;
	pos = strchr(pos, ' ');  //查询指令中第一个空格的位置
	pos++; //跳过空格
		while((const char)(*pos) != '\n')
		{
			if((const char)(*pos) != ' ')
			{
				A_photo_point = atoi(pos);  //转换成整型

				break;
			}
			else
			{
				pos++; //跳过空格
			}
		}
   err= netconn_write(newconn, databuf, len, NETCONN_COPY);

	if (err != ERR_OK)
	{
	 printf("tcpecho: netconn_write: error \"%s\"\n",
	  lwip_strerr(err));
	}
	while (netbuf_next(buf) >= 0);
	memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
  }
//  设置B轴拍照点
  	    cmdHeader = my_uart_strstr((const char *)databuf,
  	    		                              setpoint_B,
  								       sizeof(setpoint_B));
  	    end = strchr(cmdHeader, '\n');
    if(cmdHeader != NULL && end != NULL)
    {
  	pos = cmdHeader;
  	pos = strchr(pos, ' ');  //查询指令中第一个空格的位置
  	pos++; //跳过空格
  		while((const char)(*pos) != '\n')
  		{
  			if((const char)(*pos) != ' ')
  			{
  				B_photo_point = atoi(pos);  //转换成整型

  				break;
  			}
  			else
  			{
  				pos++; //跳过空格
  			}
  		}
     err= netconn_write(newconn, databuf, len, NETCONN_COPY);

  	if (err != ERR_OK)
  	{
  	 printf("tcpecho: netconn_write: error \"%s\"\n",
  	  lwip_strerr(err));
  	}
  	while (netbuf_next(buf) >= 0);
  	memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
    }
//A轴当前位置计数清零，一去一回可能存在脉冲数，需要清零
	cmdHeader = my_uart_strstr((const char *)databuf,
			                                 clear_A,
								      sizeof(clear_A));
			end = strchr(cmdHeader, '\n');
	if(cmdHeader != NULL && end != NULL)
	{
		accumulation=0;                   //中断进入次数为0
		Encoder3_Init(0,0);               //初始化定时器3
		err= netconn_write(newconn, databuf, len, NETCONN_COPY);

		if (err != ERR_OK)
		{
		 printf("tcpecho: netconn_write: error \"%s\"\n",
		  lwip_strerr(err));
		}
		while (netbuf_next(buf) >= 0);
		memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
	 }
//B轴当前位置计数清零
		cmdHeader = my_uart_strstr((const char *)databuf,
				                                 clear_B,
									      sizeof(clear_B));
				end = strchr(cmdHeader, '\n');
		if(cmdHeader != NULL && end != NULL)
		{

			TIM5->CNT = 0;   //计数器5清零
			err= netconn_write(newconn, databuf, len, NETCONN_COPY);

			if (err != ERR_OK)
			{
			 printf("tcpecho: netconn_write: error \"%s\"\n",
			  lwip_strerr(err));
			}
			while (netbuf_next(buf) >= 0);
			memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
		 }
//获取拍照A锁存值，并将拍照A锁存值发送给plc
cmdHeader = my_uart_strstr((const char *)databuf,
	         	                      getlatch_A,
							   sizeof(getlatch_A));
	end = strchr(cmdHeader, '\n');
	if(cmdHeader != NULL && end != NULL)
	{

		itoa(GETBUF[0], point_buf,10);//A轴发送
		err= netconn_write(newconn,point_buf, sizeof(point_buf), NETCONN_COPY);

		if (err != ERR_OK)
		{
		 printf("tcpecho: netconn_write: error \"%s\"\n",
		  lwip_strerr(err));
		}
//		while (netbuf_next(GETBUF) >= 0);
		memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
	}
//获取拍照B锁存值
	cmdHeader = my_uart_strstr((const char *)databuf,
		         	                      getlatch_B,
								   sizeof(getlatch_B));
		end = strchr(cmdHeader, '\n');
		if(cmdHeader != NULL && end != NULL)
		{

			itoa(GETBUF[1], point_buf,10);//B轴发送
			err= netconn_write(newconn,point_buf, sizeof(point_buf), NETCONN_COPY);

			if (err != ERR_OK)
			{
			 printf("tcpecho: netconn_write: error \"%s\"\n",
			  lwip_strerr(err));
			}
	//		while (netbuf_next(GETBUF) >= 0);
			memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据
        }


//Flag_A标志位flag_ab[0]，发送给plc，判断拍照A锁存值是否保存下来
	cmdHeader = my_uart_strstr((const char *)databuf,
		                       	        flag_point_A,
								   sizeof(flag_point_A));
		end = strchr(cmdHeader, '\n');
		if(cmdHeader != NULL && end != NULL)
		{

			itoa(flag_ab[0], point_buf,10);
			err= netconn_write(newconn,point_buf, sizeof(point_buf), NETCONN_COPY);

			if (err != ERR_OK)
			{
			 printf("tcpecho: netconn_write: error \"%s\"\n",
			  lwip_strerr(err));
			}
	//		while (netbuf_next(GETBUF) >= 0);
			memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据

		}

//Flag_B标志位
			cmdHeader = my_uart_strstr((const char *)databuf,
				                       	        flag_point_B,
										   sizeof(flag_point_B));
				end = strchr(cmdHeader, '\n');
				if(cmdHeader != NULL && end != NULL)
				{

					itoa(flag_ab[1], point_buf,10);
					err= netconn_write(newconn,point_buf, sizeof(point_buf), NETCONN_COPY);

					if (err != ERR_OK)
					{
					 printf("tcpecho: netconn_write: error \"%s\"\n",
					  lwip_strerr(err));
					}
			//		while (netbuf_next(GETBUF) >= 0);
					memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据

				}
//清空标志位
			cmdHeader = my_uart_strstr((const char *)databuf,
					                                clear_FLL,
										   sizeof(clear_FLL));
				end = strchr(cmdHeader, '\n');
				if(cmdHeader != NULL && end != NULL)
				{

//					itoa(flag_ab[1], point_buf,10);
					a=0;
					ponint_time=0;
					flag_ab[0]=0;
					flag_ab[1]=0;
					memset(point_flag,0,sizeof(point_flag));
					err= netconn_write(newconn,clear_FLL, sizeof(clear_FLL), NETCONN_COPY);

					if (err != ERR_OK)
					{
					 printf("tcpecho: netconn_write: error \"%s\"\n",
					  lwip_strerr(err));
					}
			//		while (netbuf_next(GETBUF) >= 0);
					memset((char *)cmdHeader, 0, ((int)end - (int)cmdHeader + 1 ));  //清空数据

				}








 }

//				memset((char *)cmdHeader, 0, sizeof(setpoint));


//				printf("收到ROS层以太网控制包长度：%d\n字节",len);

//			ControlPack * agv_ControlPack = (ControlPack * )databuf;

//
//			/* 如果上层下发的包头和包长度正确则开始处里*/
//			   //  if(1)
//			   //   {}
//			if(agv_ControlPack->pack_head == AGV_TCP_PACKHEADER )
//			{
//				u16_t crc16 = 0xFFFF;
//				u16_t sumlen = 0;
//				u8_t  * databuff = databuf;
//
//				//计算接收包长度
//				sumlen = agv_ControlPack->pack_length + 8;
//
//				//CRC校验
//				crc16 = HAL_CRC_Calculate ( &hcrc, (uint32_t *)&databuff[4], (sumlen - 6) ); //sizeof(ControlPack) - 4
////
////				/* 校验通过，开始根据下发的指令进行处理 */
//				if( crc16 == agv_ControlPack -> crc16_t )
//				{
//
//					switch( agv_ControlPack->funtion_code )
//					{
//						//设置拍照点
//						case SET_PHOTO_PIONT1:
//						{
//							photo_point = agv_ControlPack->aaa;
//							send_data_len = DataReturn(SET_PHOTO_PIONT1, photo_point);
//							err = netconn_write(newconn, tcpDataArray, send_data_len, NETCONN_NOCOPY);
//							if (err != ERR_OK)
//							{
//								printf("err=%d\n",err );
//							}
//
//						}; break;
//						case SET_PHOTO_PIONT2:
//						{
//
//
//						}; break;
//						default:; break;
//					}
//				}
//			}

			/* 清理netbuf空间，等待下一次数据连接*/
			netbuf_delete(buf);
		}
	 }

	 /* 断开TCP连接，删除netconn结构体，等待下一次握手*/
	 netconn_close(newconn);
	 netconn_delete(newconn);
 }
}


/*-----------------------------------------------------------------------------------*/
extern TaskHandle_t ROS_TCP_TASK_Handle;	/* ROS_TCP任务句柄 */
void tcpecho_init(void)
{
	ROS_TCP_TASK_Handle = sys_thread_new( "cmdTCP_thread", tcpcmd_thread, NULL, 2048, 8 );
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */
