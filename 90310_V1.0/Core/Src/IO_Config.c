#include "main.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/* 以太网 */
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "LAN8720a.h"

#include "IO_Config.h"


void ST26C32_EN_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

/*-------------- 差分转单端信号芯片使能引脚 -----------------*/
    ST26C32_EN1_GPIO_CLK_ENABLE();
    ST26C32_EN2_GPIO_CLK_ENABLE();
    /*输入IO的配置*/
    __HAL_RCC_GPIOB_CLK_ENABLE();
//EN1
    GPIO_InitStruct.Pin = ST26C32_EN1_GPIO_PIN;
    /* 设置引脚为输入模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    /* 设置引脚上拉 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	/*设置引脚速率为高速 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ST26C32_EN1_GPIO_PORT, &GPIO_InitStruct);

//EN2
    GPIO_InitStruct.Pin = ST26C32_EN2_GPIO_PIN;
    HAL_GPIO_Init(ST26C32_EN2_GPIO_PORT, &GPIO_InitStruct);

    ST26C32_EN1(1);		//高或低电平都是使能
    ST26C32_EN2(1);


//MCU_OUTPUT1
    MCU_OUTPUT1_GPIO_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin = MCU_OUTPUT1_GPIO_PIN;
	/* 设置引脚为输入模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	/* 设置引脚上拉 */
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	/*设置引脚速率为高速 */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MCU_OUTPUT1_GPIO_PORT, &GPIO_InitStruct);
	/*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1
	                          |GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

	/* 下限位开关IO时钟 */
//	DS_Limit_DOWN_GPIO_CLK_ENABLE() ;

	/* 下限位开关输入中断引脚配置 */
	/* 下限位开关的引脚 */
//	GPIO_InitStructure.Pin = DS_Limit_DOWN_GPIO_PIN;
	/* 设置引脚为输入模式 */
//	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
	/* 设置引脚上拉 */
//	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	/* 下限位开关初始化 */
//	HAL_GPIO_Init(DS_/Limit_DOWN_GPIO_PORT, &GPIO_InitStructure);
	/* 配置中断优先级*/
//	HAL_NVIC_SetPriority(DS_Limit_DOWN_EXTI_IRQ, 5, 0);
//	/* 使能中断 */
//	HAL_NVIC_EnableIRQ(DS_Limit_DOWN_EXTI_IRQ);

	/*Configure GPIO pins : PD10 PD11 PD0 PD1
	                           PD2 PD3 PD4 PD5
	                           PD6 PD7 */
	  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1
	                          |GPIO_PIN_2|GPIO_PIN_3;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


	  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
	    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;//
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    /*Configure GPIO pin : PB6 */
	      GPIO_InitStruct.Pin = GPIO_PIN_6;
	      GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	      GPIO_InitStruct.Pull = GPIO_NOPULL;
	      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	      /*Configure GPIO pin : PB7 */
		  GPIO_InitStruct.Pin = GPIO_PIN_7;
		  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		  GPIO_InitStruct.Pull = GPIO_NOPULL;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


}


//void GPIO_PinReverse(GPIO_TypeDef*GPIOx,uint16_t GPIO_pin)
//{
//   assert_param (IS_GPIO_ALL_PERIPH(GPIOx));
//   assert_param (IS_GPIO_PIN(GPIO_Pin));
//   GPIOx->ODR ^= GPIO_pin;
//}
