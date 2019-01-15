#ifndef  __SYSTEM_TIMER_H__
#define  __SYSTEM_TIMER_H__
#include "bsp_include.h"
#define            SYSTEM_TIM                   TIM2
#define            SYSTEM_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            SYSTEM_TIM_CLK               RCC_APB1Periph_TIM2
#define            SYSTEM_TIM_Period            (1000-1)
#define            SYSTEM_TIM_Prescaler         71
#define            SYSTEM_TIM_IRQ               TIM2_IRQn
#define            SYSTEM_TIM_IRQHandler        TIM2_IRQHandler



extern uint32_t time_1ms, time_10ms,time_20ms,time_50ms,time_100ms,time_200ms, time_1000ms ,time_5000ms ,time_30ms,time_continue; // ms 计时变量















/*                    外部函数声明            */
void SYSTEM_TIM_Init(uint16_t Frequency);







#endif //__SYSTEM_TIMER_H__

