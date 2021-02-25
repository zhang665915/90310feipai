#include "main.h"

/* DSP库 */
#include "math.h"
#include "arm_math.h"

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

/* LWIP头文件*/
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/opt.h"

#include "usart.h"
#include "tcpecho.h"
#include "udpecho.h"
#include "mcu_ulp_mode.h"
#include "stdio.h"
#include "encoder.h"
#include "tjshow.h"
#include "user_flash.h"


/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */

//SemaphoreHandle_t NodeOnlineSem_Handle = NULL;		/* 创建二值信号量，用于节点上线*/
EventGroupHandle_t PWM_Start_Event = NULL; //创建开启PWM事件
EventGroupHandle_t Rx_Set_Freq_Event = NULL; //接收串口屏设置频率事件

/* freeRTOS相关句柄 */
static TaskHandle_t AppTaskCreate_Handle = NULL;			/* 创建任务句柄 */
static TaskHandle_t Get_Encoder_Task_Handle = NULL;			/* 创建获取脉冲数任务句柄 */
static TaskHandle_t Tjshow_Task_Handle = NULL;				/* 创建串口屏显示任务句柄 */
static TaskHandle_t Produce_PWM_Task_Handle = NULL;			/* 产生控制继电器的PWM任务句柄 */
static TaskHandle_t PWM_Start_Task_Handle = NULL;			/* 启动或关闭PWM输出任务句柄  */
//static TaskHandle_t System_Managem_Task_Handle = NULL;		/* 系统管理任务句柄 */

TaskHandle_t ROS_TCP_TASK_Handle = NULL;	/* ROS_TCP任务句柄 */
TaskHandle_t ROS_UDP_TASK_Handle = NULL;	/* ROS_UDP任务句柄 */


/* 消息队列的长度的消息单元大小 */
#define QUEUE_LEN   4
#define QUEUE_SIZE  4

static void AppTaskCreate(void);/* 用于创建任务 */
extern void TCPIP_Init(void);


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{



	BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */

    /* 开发板硬件初始化 */
    BSP_Init();

    /* 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t	) AppTaskCreate, /* 任务入口函数 */
    						(const char*	) "AppTaskCreate",/* 任务名字 */
							(uint16_t		) 512, /* 任务栈大小 */
							(void*			) NULL,/* 任务入口函数参数 */
							(UBaseType_t	) 1, /* 任务的优先级 */
							(TaskHandle_t*	) &AppTaskCreate_Handle);/* 任务控制块指针 */
    /* 启动任务调度 */
    if (pdPASS == xReturn)
    {
        printf("Create AppTaskCreate_Task sucess...\r\n");
        vTaskStartScheduler(); /* 启动任务，开启调度 */
    }
    else
        return -1;

    while (1)
        ; /* 正常不会执行到这里 */
}

/***********************************************************************
 * @ 函数名  ： AppTaskCreate
 * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
 * @ 参数    ： 无
 * @ 返回值  ： 无
 **********************************************************************/
static void AppTaskCreate(void)
{

    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */

    PWM_Start_Event = xEventGroupCreate();
    Rx_Set_Freq_Event = xEventGroupCreate();

    taskENTER_CRITICAL();           //进入临界区

    /* 创建TCP/IP内核线程 */
   TCPIP_Init();

    /* 创建TCP任务 */
   tcpecho_init();

    /* 创建UDP任务 */
//    udpecho_init();

	/* 创建系统管理任务 */
//    xReturn = xTaskCreate((TaskFunction_t) System_Managem_Task, /* 任务入口函数 */
//    					  (const char*	 ) "System_Managem_Task",/* 任务名字 */
//						  (uint16_t		 ) 512, /* 任务栈大小 */
//						  (void*		 ) NULL, /* 任务入口函数参数 */
//						  (UBaseType_t	 ) 7, /* 任务的优先级 */
//						  (TaskHandle_t* ) &System_Managem_Task_Handle);/* 任务控制块指针 */
//	if (pdPASS == xReturn)
//        printf("Create System_Managem_Task sucess...\r\n");

		/* 创建获取脉冲数任务 */
//	    xReturn = xTaskCreate((TaskFunction_t) Get_Encoder_Task, /* 任务入口函数 */
//	    					  (const char*	 ) "Get_Encoder_Task",/* 任务名字 */
//							  (uint16_t		 ) 512, /* 任务栈大小 */
//							  (void*		 ) NULL, /* 任务入口函数参数 */
//							  (UBaseType_t	 ) 8, /* 任务的优先级 */
//							  (TaskHandle_t* ) &Get_Encoder_Task_Handle);/* 任务控制块指针 */
//		if (pdPASS == xReturn)
//	        printf("Create Get_Encoder_Task sucess...\r\n");

		/* 创建产生控制继电器的PWM任务 */
//		xReturn = xTaskCreate((TaskFunction_t) Produce_PWM_Task, /* 任务入口函数 */
//							  (const char*	 ) "Produce_PWM_Task",/* 任务名字 */
//							  (uint16_t		 ) 512, /* 任务栈大小 */
//							  (void*		 ) NULL, /* 任务入口函数参数 */
//							  (UBaseType_t	 ) 8, /* 任务的优先级 */
//							  (TaskHandle_t* ) &Produce_PWM_Task_Handle);/* 任务控制块指针 */
//		if (pdPASS == xReturn)
//			printf("Create Produce_PWM_Task sucess...\r\n");

		/* 创建串口屏显示任务 */
//		xReturn = xTaskCreate((TaskFunction_t) Tjshow_Task, /* 任务入口函数 */
//							  (const char*	 ) "Tjshow_Task",/* 任务名字 */
//							  (uint16_t		 ) 512, /* 任务栈大小 */
//							  (void*		 ) NULL, /* 任务入口函数参数 */
//							  (UBaseType_t	 ) 7, /* 任务的优先级 */
//							  (TaskHandle_t* ) &Tjshow_Task_Handle);/* 任务控制块指针 */
//		if (pdPASS == xReturn)
//			printf("Create Tjshow_Task sucess...\r\n");

		/* 创建启动或关闭PWM输出任务 */
//		xReturn = xTaskCreate((TaskFunction_t) PWM_Start_Task, /* 任务入口函数 */
//							  (const char*	 ) "PWM_Start_Task",/* 任务名字 */
//							  (uint16_t		 ) 512, /* 任务栈大小 */
//							  (void*		 ) NULL, /* 任务入口函数参数 */
//							  (UBaseType_t	 ) 7, /* 任务的优先级 */
//							  (TaskHandle_t* ) &PWM_Start_Task_Handle);/* 任务控制块指针 */
//		if (pdPASS == xReturn)
//			printf("Create PWM_Start_Task sucess...\r\n");



    vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务

    taskEXIT_CRITICAL();            //退出临界区
}



/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1);
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
