#ifndef __LED_H
#define	__LED_H

#include "stm32h7xx.h"


#define USER_REDLED_FOR_POWER_INDICATOR 	1			//使用该定义，红色指示灯用于电源指示灯，在此表示，系统一运行，就点亮该灯，一直亮。

//引脚定义---- 野火挑战者的三色灯---PH10--R, PH11--G, PH12--B
/*******************************************************/
//R 红色灯
#define LED1_PIN                  GPIO_PIN_10                 
#define LED1_GPIO_PORT            GPIOH                      
#define LED1_GPIO_CLK_ENABLE()    __GPIOH_CLK_ENABLE()

//G 绿色灯
#define LED2_PIN                  GPIO_PIN_11                
#define LED2_GPIO_PORT            GPIOH                      
#define LED2_GPIO_CLK_ENABLE()    __GPIOH_CLK_ENABLE()

//B 蓝色灯
#define LED3_PIN                  GPIO_PIN_12                 
#define LED3_GPIO_PORT            GPIOH                       
#define LED3_GPIO_CLK_ENABLE()    __GPIOH_CLK_ENABLE()


//报警三色灯宏定义（6个灯）
//R 红色灯 急停（停止）
#define LED_STOP_KEY_PIN                  GPIO_PIN_6                 
#define LED_STOP_KEY_GPIO_PORT            GPIOB                     
#define LED_STOP_KEY_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
#define LED_STOP_ALARM_PIN                  GPIO_PIN_3                 
#define LED_STOP_ALARM_GPIO_PORT            GPIOI                      
#define LED_STOP_ALARM_GPIO_CLK_ENABLE()    __GPIOI_CLK_ENABLE()

//G 绿色灯 启动（继续）
#define LED_CONTINUE_KEY_PIN                  GPIO_PIN_12                 
#define LED_CONTINUE_KEY_GPIO_PORT            GPIOB                      
#define LED_CONTINUE_KEY_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()
#define LED_CONTINUE_ALARM_PIN                  GPIO_PIN_7                 
#define LED_CONTINUE_ALARM_GPIO_PORT            GPIOB                      
#define LED_CONTINUE_ALARM_GPIO_CLK_ENABLE()    __GPIOB_CLK_ENABLE()

//B 黄色灯  停止 （按键灯是白灯）（暂停）
#define LED_PAUSE_KEY_PIN                  GPIO_PIN_2                 
#define LED_PAUSE_KEY_GPIO_PORT            GPIOC                      
#define LED_PAUSE_KEY_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()
#define LED_PAUSE_ALARM_PIN                  GPIO_PIN_1                 
#define LED_PAUSE_ALARM_GPIO_PORT            GPIOI                      
#define LED_PAUSE_ALARM_GPIO_CLK_ENABLE()    __GPIOI_CLK_ENABLE()

//W Free  （备用LED端口）
#define LED_FREE_KEY_PIN                  	 GPIO_PIN_2   // 备用按键灯              
#define LED_FREE_KEY_GPIO_PORT            	 GPIOE                     
#define LED_FREE_KEY_GPIO_CLK_ENABLE()       __GPIOE_CLK_ENABLE()

//BEEP  （蜂鸣器端口）
#define LED_ALARM_BEEP_PIN                 	GPIO_PIN_6               
#define LED_ALARM_BEEP_GPIO_PORT            GPIOC                     
#define LED_ALARM_BEEP_GPIO_CLK_ENABLE()    __GPIOC_CLK_ENABLE()




//小指示灯
//#define LED4_PIN                  GPIO_PIN_3                 
//#define LED4_GPIO_PORT            GPIOA                       
//#define LED4_GPIO_CLK_ENABLE()    __GPIOA_CLK_ENABLE()
/************************************************************/


/** 控制LED灯亮灭的宏，
	* LED低电平亮，设置ON=0，OFF=1
	* 若LED高电平亮，把宏设置成ON=1 ，OFF=0 即可
	*/
#define ON  GPIO_PIN_RESET
#define OFF GPIO_PIN_SET

/* 带参宏，可以像内联函数一样使用 */
#define LED1(a)	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_PIN,a)


#define LED2(a)	HAL_GPIO_WritePin(LED2_GPIO_PORT,LED2_PIN,a)


#define LED3(a)	HAL_GPIO_WritePin(LED2_GPIO_PORT,LED3_PIN,a)


#define LED4(a)	HAL_GPIO_WritePin(LED4_GPIO_PORT,LED4_PIN,a)


/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)				{p->BSRR=i;}			  //设置为高电平		
#define digitalLo(p,i)				{p->BSRR=(uint32_t)i << 16;}					//输出低电平
#define digitalToggle(p,i)			{p->ODR ^=i;}			//输出反转状态


/* 定义控制IO的宏 */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF			digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF			digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF			digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON				digitalLo(LED3_GPIO_PORT,LED3_PIN)

#define LED4_TOGGLE		digitalToggle(LED4_GPIO_PORT,LED4_PIN)
#define LED4_OFF			digitalHi(LED4_GPIO_PORT,LED4_PIN)
#define LED4_ON				digitalLo(LED4_GPIO_PORT,LED4_PIN)


//报警灯和板载灯用
//急停
#define LED_STOP_KEY_TOGGLE					digitalToggle(LED_STOP_KEY_GPIO_PORT,LED_STOP_KEY_PIN)

#ifndef USER_REDLED_FOR_POWER_INDICATOR
#define LED_STOP_KEY_OFF						digitalHi(LED_STOP_KEY_GPIO_PORT,LED_STOP_KEY_PIN)
#define LED_STOP_KEY_ON							digitalLo(LED_STOP_KEY_GPIO_PORT,LED_STOP_KEY_PIN)
#else
#define LED_STOP_KEY_OFF						
#define LED_STOP_KEY_ON		
#define LED_POWER_OFF							digitalHi(LED_STOP_KEY_GPIO_PORT,LED_STOP_KEY_PIN)					
#define LED_POWER_ON							digitalLo(LED_STOP_KEY_GPIO_PORT,LED_STOP_KEY_PIN)
#endif

#define LED_STOP_ALARM_TOGGLE					digitalToggle(LED_STOP_ALARM_GPIO_PORT,LED_STOP_ALARM_PIN)
#define LED_STOP_ALARM_OFF						digitalHi(LED_STOP_ALARM_GPIO_PORT,LED_STOP_ALARM_PIN)
#define LED_STOP_ALARM_ON							digitalLo(LED_STOP_ALARM_GPIO_PORT,LED_STOP_ALARM_PIN)

//启动
#define LED_CONTINUE_KEY_TOGGLE			digitalToggle(LED_CONTINUE_KEY_GPIO_PORT,LED_CONTINUE_KEY_PIN)
#define LED_CONTINUE_KEY_OFF				digitalHi(LED_CONTINUE_KEY_GPIO_PORT,LED_CONTINUE_KEY_PIN)
#define LED_CONTINUE_KEY_ON				digitalLo(LED_CONTINUE_KEY_GPIO_PORT,LED_CONTINUE_KEY_PIN)
#define LED_CONTINUE_ALARM_TOGGLE			digitalToggle(LED_CONTINUE_ALARM_GPIO_PORT,LED_CONTINUE_ALARM_PIN)
#define LED_CONTINUE_ALARM_OFF				digitalHi(LED_CONTINUE_ALARM_GPIO_PORT,LED_CONTINUE_ALARM_PIN)
#define LED_CONTINUE_ALARM_ON				digitalLo(LED_CONTINUE_ALARM_GPIO_PORT,LED_CONTINUE_ALARM_PIN)

//暂停
#define LED_PAUSE_KEY_TOGGLE				digitalToggle(LED_PAUSE_KEY_GPIO_PORT,LED_PAUSE_KEY_PIN)
#define LED_PAUSE_KEY_OFF					digitalHi(LED_PAUSE_KEY_GPIO_PORT,LED_PAUSE_KEY_PIN)
#define LED_PAUSE_KEY_ON					digitalLo(LED_PAUSE_KEY_GPIO_PORT,LED_PAUSE_KEY_PIN)
#define LED_PAUSE_ALARM_TOGGLE				digitalToggle(LED_PAUSE_ALARM_GPIO_PORT,LED_PAUSE_ALARM_PIN)
#define LED_PAUSE_ALARM_OFF					digitalHi(LED_PAUSE_ALARM_GPIO_PORT,LED_PAUSE_ALARM_PIN)
#define LED_PAUSE_ALARM_ON					digitalLo(LED_PAUSE_ALARM_GPIO_PORT,LED_PAUSE_ALARM_PIN)


//备用LED端口---PE2
#define LED_FREE_KEY_TOGGLE					digitalToggle(LED_FREE_KEY_GPIO_PORT,LED_FREE_KEY_PIN)
#define LED_FREE_KEY_OFF					digitalHi(LED_FREE_KEY_GPIO_PORT,LED_FREE_KEY_PIN)
#define LED_FREE_KEY_ON						digitalLo(LED_FREE_KEY_GPIO_PORT,LED_FREE_KEY_PIN)


//蜂鸣器端口---PC6
#define LED_ALARM_BEEP_TOGGLE				digitalToggle(LED_ALARM_BEEP_GPIO_PORT,LED_ALARM_BEEP_PIN)
#define LED_ALARM_BEEP_OFF					digitalHi(LED_ALARM_BEEP_GPIO_PORT,LED_ALARM_BEEP_PIN)
#define LED_ALARM_BEEP_ON					digitalLo(LED_ALARM_BEEP_GPIO_PORT,LED_ALARM_BEEP_PIN)



//按键灯  与 报警灯 同步
//工步运行 绿灯
#define LED_CONTINUE  \
					LED_CONTINUE_KEY_ON;\
					LED_CONTINUE_ALARM_ON;\
					LED_PAUSE_KEY_OFF;\
					LED_PAUSE_ALARM_OFF;\
					LED_STOP_KEY_OFF;\
					LED_STOP_ALARM_OFF;

//工步暂停 黄灯 （按键是白灯）
#define LED_PAUSE  \
					LED_CONTINUE_KEY_OFF;\
					LED_CONTINUE_ALARM_OFF;\
					LED_PAUSE_KEY_ON;\
					LED_PAUSE_ALARM_ON;\
					LED_STOP_KEY_OFF;\
					LED_STOP_ALARM_OFF;\

#define LED_FREE  \
					LED_CONTINUE_KEY_OFF;\
					LED_CONTINUE_ALARM_OFF;\
					LED_FREE_KEY_ON;\
					LED_PAUSE_ALARM_ON;\
					LED_STOP_KEY_OFF;\
					LED_STOP_ALARM_OFF;


//工步停止 红灯
#define LED_STOP  \
					LED_CONTINUE_KEY_OFF;\
					LED_CONTINUE_ALARM_OFF;\
					LED_PAUSE_KEY_OFF;\
					LED_PAUSE_ALARM_OFF;\
					LED_STOP_KEY_ON;\
					LED_STOP_ALARM_ON;

//工步结束 灯全灭
#define LED_OFF  \
					LED_CONTINUE_KEY_OFF;\
					LED_CONTINUE_ALARM_OFF;\
					LED_PAUSE_KEY_OFF;\
					LED_PAUSE_ALARM_OFF;\
					LED_STOP_KEY_OFF;\
					LED_STOP_ALARM_OFF;\
					LED_FREE_KEY_OFF;\
					LED_ALARM_BEEP_OFF;


#define LED_BEEP  \
					LED_CONTINUE_KEY_OFF;\
					LED_CONTINUE_ALARM_OFF;\
					LED_FREE_KEY_OFF;\
					LED_PAUSE_ALARM_OFF;\
					LED_STOP_KEY_OFF;\
					LED_STOP_ALARM_OFF;\
					LED_ALARM_BEEP_ON;





/* 基本混色，后面高级用法使用PWM可混出全彩颜色,且效果更好 */

//红
#define LED_RED  \
					LED1_ON;\
					LED2_OFF\
					LED3_OFF

//绿
#define LED_GREEN		\
					LED1_OFF;\
					LED2_ON\
					LED3_OFF

//蓝
#define LED_BLUE	\
					LED1_OFF;\
					LED2_OFF\
					LED3_ON

					
//黄(红+绿)					
#define LED_YELLOW	\
					LED1_ON;\
					LED2_ON\
					LED3_OFF
//紫(红+蓝)
#define LED_PURPLE	\
					LED1_ON;\
					LED2_OFF\
					LED3_ON

//青(绿+蓝)
#define LED_CYAN \
					LED1_OFF;\
					LED2_ON\
					LED3_ON
					
//白(红+绿+蓝)
#define LED_WHITE	\
					LED1_ON;\
					LED2_ON\
					LED3_ON
					
//黑(全部关闭)
#define LED_RGBOFF	\
					LED1_OFF;\
					LED2_OFF\
					LED3_OFF
					




void LED_GPIO_Config(void);

#endif /* __LED_H */
