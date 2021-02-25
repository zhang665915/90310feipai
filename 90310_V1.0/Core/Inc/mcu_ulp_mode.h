
#ifndef STM32H7_ULP_
#define STM32H7_ULP_

#include "stm32h7xx.h"

#define RTC_ULP_PROTECT				0	//关闭RTC的相关中断

//唤醒引脚配置
#define WKUP_GPIO_PORT              GPIOA
#define WKUP_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE();
#define WKUP_GPIO_PIN               GPIO_PIN_0
#define WKUP_EXTI_IRQ               EXTI0_IRQn
#define WKUP_IRQHandler             EXTI0_IRQHandler

//唤醒引脚对应的 wake_up 编号 1-6
#define PWR_WAKEUP_PIN				PWR_WAKEUP_PIN1

//唤醒引脚的状态
#define WKUP_STATUS					HAL_GPIO_ReadPin(WKUP_GPIO_PORT, WKUP_GPIO_PIN)



void Sys_Enter_Standby(void);
uint8_t Check_WKUP(void);
void WKUP_Init(void);
void System_Managem_Task( void *paramter );



#endif
