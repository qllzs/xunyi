#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__
#include "bsp_include.h"

/*看门狗宏定义*/
//脉冲看门狗数组编号
#define PULSE_WATCHDOG   0
//脉冲看门狗定时时间为单位为 （PULSE_WDT_TIMEOUT * 每次调用时间间隔）ms
#define PULSE_WDT_TIMEOUT 1000



#define            WDG_TIM                   TIM5
#define            WDG_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            WDG_TIM_CLK               RCC_APB1Periph_TIM5
#define            WDG_TIM_Period            (1000-1)
#define            WDG_TIM_Prescaler         71
#define            WDG_TIM_IRQ               TIM5_IRQn
#define            WDG_TIM_IRQHandler        TIM5_IRQHandler


/*————————————外部变量声明——————————*/


extern uint16_t WatchDog[5] ;
extern uint8_t NoPulseFlag;





/*————————————外部函数声明——————————*/


void IWDG_Feed(void);
void IWDG_Config(uint8_t prv ,uint16_t rlv);
void WDG_TIM_Init(uint16_t Frequency);




#endif   //__WATCHDOG_H__


