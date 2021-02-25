#include "key.h"
//#include "delay.h"
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

//按键初始化函数
//void KEY_Init(void)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//
//	__HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
//    __HAL_RCC_GPIOC_CLK_ENABLE();           //开启GPIOC时钟
//    __HAL_RCC_GPIOH_CLK_ENABLE();           //开启GPIOH时钟
//
//    GPIO_Initure.Pin=GPIO_PIN_0;            //PA0
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入
//    GPIO_Initure.Pull=GPIO_PULLDOWN;        //下拉
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
//
//    GPIO_Initure.Pin=GPIO_PIN_13;           //PC13
//    GPIO_Initure.Mode=GPIO_MODE_INPUT;      //输入

//    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //高速
//    HAL_GPIO_Init(GPIOC,&GPIO_Initure);
//
//    GPIO_Initure.Pin=GPIO_PIN_2|GPIO_PIN_3; //PH2,3
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);
//}
//
////按键处理函数
////返回按键值
////mode:0,不支持连续按;1,支持连续按;
////0，没有任何按键按下
////1，WKUP按下 WK_UP
////注意此函数有响应优先级,KEY0>KEY1>KEY2>WK_UP!!
//uint8_t KEY_Scan(uint8_t mode)
//{
//    static uint8_t key_up=1;     //按键松开标志
//    if(mode==1)key_up=1;    //支持连按
//    if(key_up&&(KEY0==0||KEY1==0||KEY2==0||WK_UP==1))
//    {
//        //delay_ms(10);
//        HAL_Delay(10);
//        key_up=0;
//        if(KEY0==0)       return KEY0_PRES;
//        else if(KEY1==0)  return KEY1_PRES;
//        else if(KEY2==0)  return KEY2_PRES;
//        else if(WK_UP==1) return WKUP_PRES;
//    }else if(KEY0==1&&KEY1==1&&KEY2==1&&WK_UP==0)key_up=1;
//    return 0;   //无按键按下
//}

/////////////////////////////野火按键检测///////////////////////////////////
/**
  * @brief  配置按键用到的I/O口
  * @param  无
  * @retval 无
  */
void Key_GPIO_Config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	/*开启按键GPIO口的时钟*/
	KEY1_GPIO_CLK_ENABLE();
	KEY2_GPIO_CLK_ENABLE();
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = KEY1_PIN;
	/*设置引脚为输入模式*/
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	/*设置引脚不上拉也不下拉*/
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(KEY1_GPIO_PORT, &GPIO_InitStructure);
	/*选择按键的引脚*/
	GPIO_InitStructure.Pin = KEY2_PIN;
	/*使用上面的结构体初始化按键*/
	HAL_GPIO_Init(KEY2_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief   检测是否有按键按下
  * @param   具体的端口和端口位
  *		@arg GPIOx: x可以是（A...G）
  *		@arg GPIO_PIN 可以是GPIO_PIN_x（x可以是1...16）
  * @retval  按键的状态
  *		@arg KEY_ON:按键按下
  *		@arg KEY_OFF:按键没按下
  */

uint8_t Key_Scan(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin)

{

	/*检测是否有按键按下 */
	if(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON )
	{
		/*等待按键释放 */
		while(HAL_GPIO_ReadPin(GPIOx,GPIO_Pin) == KEY_ON);
		return 	KEY_ON;
	}
	else
		return KEY_OFF;
}
