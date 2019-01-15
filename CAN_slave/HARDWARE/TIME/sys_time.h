#ifndef __SYS_TIME_H
#define __SYS_TIME_H
#include "GlobalParam.h" 

#define        SYSTEM_TIM_Prescaler         7200

void SYSTEM_TIM_NVIC_Config(void);
void System_TimeInit(uint16_t	Freq);
#endif
