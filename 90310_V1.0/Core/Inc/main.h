#ifndef __MAIN_H
#define __MAIN_H

#include "stm32h7xx_hal.h"



void BSP_Init(void);
void AGVServoTaskCreate(void);
void AGVServoTaskDelete(void);
void Error_Handler(void);



#endif /* __MAIN_H */

