#ifndef _KEY_H
#define _KEY_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32H7开发板
//KEY驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/6/8
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//#define KEY0        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_3)  //KEY0按键PH3
//#define KEY1        HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_2)  //KEY1按键PH2
//#define KEY2        HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) //KEY2按键PC13
//#define WK_UP       HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)  //WKUP按键PA0
//
//#define KEY0_PRES 	1  	//KEY0按下后返回值
//#define KEY1_PRES	2	//KEY1按下后返回值
//#define KEY2_PRES	3	//KEY2按下后返回值
//#define WKUP_PRES   4	//WKUP按下后返回值
//
//void KEY_Init( void );  //按键IO初始化函数
//uint8_t KEY_Scan( uint8_t mode ); //按键扫描函数
//#endif



/////////////////////////////野火按键检测///////////////////////////////////

//引脚定义
/*******************************************************/
#define KEY1_PIN                  GPIO_PIN_0
#define KEY1_GPIO_PORT            GPIOA
#define KEY1_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()

#define KEY2_PIN                  GPIO_PIN_13
#define KEY2_GPIO_PORT            GPIOC
#define KEY2_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()
/*******************************************************/

 /** 按键按下标置宏
	* 按键按下为高电平，设置 KEY_ON=1， KEY_OFF=0
	* 若按键按下为低电平，把宏设置成KEY_ON=0 ，KEY_OFF=1 即可
	*/
#define KEY_ON	1
#define KEY_OFF	0

void Key_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin);

#endif
