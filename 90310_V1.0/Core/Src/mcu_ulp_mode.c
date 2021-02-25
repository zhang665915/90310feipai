
#include "mcu_ulp_mode.h"

/* freeRTOS */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

//系统进入待机模式
void Sys_Enter_Standby(void)
{
	__HAL_RCC_AHB1_FORCE_RESET(); //复位所有 IO口

	while(WKUP_STATUS){};

	//失能 WK_UP 引脚作为唤醒源
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN);

	__HAL_RCC_BACKUPRESET_FORCE(); 	//复位备份区域
	HAL_PWR_EnableBkUpAccess(); 	//后备区域访问使能:（备份SRAM、 RTC、 LSE）
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

#if( RTC_ULP_PROTECT == 1 )
	__HAL_RTC_WRITEPROTECTION_DISABLE(&RTC_Handler);//关闭 RTC 写保护
	//关闭 RTC 相关中断，可能在 RTC 实验打开了
	__HAL_RTC_WAKEUPTIMER_DISABLE_IT(&RTC_Handler,RTC_IT_WUT);
	__HAL_RTC_TIMESTAMP_DISABLE_IT(&RTC_Handler,RTC_IT_TS);
	__HAL_RTC_ALARM_DISABLE_IT(&RTC_Handler,RTC_IT_ALRA|RTC_IT_ALRB);
	//清除 RTC 相关中断标志位
	__HAL_RTC_ALARM_CLEAR_FLAG(&RTC_Handler,RTC_FLAG_ALRAF|RTC_FLAG_ALRBF);
	__HAL_RTC_TIMESTAMP_CLEAR_FLAG(&RTC_Handler,RTC_FLAG_TSF);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&RTC_Handler,RTC_FLAG_WUTF);
#endif
	__HAL_RCC_BACKUPRESET_RELEASE(); //备份区域复位结束
#if( RTC_ULP_PROTECT == 1 )
	__HAL_RTC_WRITEPROTECTION_ENABLE(&RTC_Handler); //使能 RTC 写保护
#endif
	//设置 WK_UP 引脚作为唤醒源
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN);
	//进入待机模式
	HAL_PWR_EnterSTANDBYMode();
}


//检测 WKUP 脚的信号
//返回值 1:连续按下 3s 以上 0:错误的触发
uint8_t Check_WKUP(void)
{
	uint8_t t = 0;	//记录按下的次数
	uint8_t tx = 0;	//记录松开的次数：松开>90ms判定为松开
	while(1)
	{
		if(WKUP_STATUS == 0)	//已经按下了
		{
			t++; tx=0;
		}
		else
		{
			tx++;
		}
		HAL_Delay(30);

		if(tx > 3)		//超过 90ms 内没有 WKUP 信号
			return 0;

		if(t >= 100)	//按下超过 3 秒钟
			return 1; 	//按下 3s 以上了
	}
}


//PA0 WKUP 唤醒初始化
void WKUP_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	WKUP_GPIO_CLK_ENABLE(); //开启WKUP 时钟
	GPIO_Initure.Pin = WKUP_GPIO_PIN;
	GPIO_Initure.Mode = GPIO_MODE_IT_FALLING; //中断,下降沿
	GPIO_Initure.Pull = GPIO_PULLUP; 	//上拉
	HAL_GPIO_Init(WKUP_GPIO_PORT, &GPIO_Initure);

	//检查是否是正常开机
	if(Check_WKUP() == 0)
	{
		Sys_Enter_Standby();//没有长按3s，不开机进入待机模式
	}
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/**
 * @brief  MCU系统管理：低功耗、看门狗喂狗等
 * @param  无
 * @retval void
 */
void System_Managem_Task( void *paramter )
{
	uint32_t system_time = 0, wk_up_time = 0 ;
	/* 用于保存上次时间，调用后系统自动更新 */
	static portTickType PreviousWakeTime;
	/* 设置延时时间，将时间转为节拍数 */
	const portTickType TimeIncrement = pdMS_TO_TICKS(50);
	/* 获取当前系统时间 */
	PreviousWakeTime = xTaskGetTickCount();

	while(1)
	{
		/* 调用绝对延时函数,任务时间间隔为 50 个 tick */
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);
		system_time = xTaskGetTickCount();	//获取当前系统时间

		//如果长按WKUP超过3s则进入低功耗模式
		if(WKUP_STATUS == 0)
		{
			wk_up_time = system_time;
		}
		else
		{
			if( system_time - wk_up_time >= 3000 )
			{
				Sys_Enter_Standby();	//进入低功耗模式
			}
		}
	}
}



