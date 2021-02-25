/**
  ******************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   led应用函数接口
  ******************************************************************
  * @attention
  *
  * 实验平台:野火 STM32H743开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************
  */  
  
#include "bsp_led.h"

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*开启LED相关的GPIO外设时钟*/
	LED1_GPIO_CLK_ENABLE();
	LED2_GPIO_CLK_ENABLE();
	LED3_GPIO_CLK_ENABLE();
	

	//90030 test
	__GPIOI_CLK_ENABLE();



	/*选择要控制的GPIO引脚*/															   
	GPIO_InitStruct.Pin = LED1_PIN;	
	
	/*设置引脚的输出类型为推挽输出*/
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;  

	/*设置引脚为上拉模式*/
	GPIO_InitStruct.Pull  = GPIO_PULLUP;

	/*设置引脚速率为高速 */   
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; 

	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
	HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);	

	/*选择要控制的GPIO引脚*/															   
	GPIO_InitStruct.Pin = LED2_PIN;	
	HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);	

	/*选择要控制的GPIO引脚*/															   
	GPIO_InitStruct.Pin = LED3_PIN;	
	HAL_GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStruct);	
	
	


//	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);


	/*关闭RGB灯*/
	LED_RGBOFF;
	LED_OFF;


	#ifdef USER_REDLED_FOR_POWER_INDICATOR
		LED_POWER_ON;
	#endif	
}
/*********************************************END OF FILE**********************/

