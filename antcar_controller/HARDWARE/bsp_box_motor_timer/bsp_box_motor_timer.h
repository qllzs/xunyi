#ifndef __BSP_BOX_MOTOR_TIMER_H__
#define __BSP_BOX_MOTOR_TIMER_H__
#include "bsp_include.h"

/*----------------------------------控制引脚宏定义---------------------------------------------*/
//通过使用寻线功能用到的引脚来控制箱门电机的正反装
#define  BOX_OPEN_GPIO_Pin          GPIO_Pin_10
#define  BOX_CLOSE_GPIO_Pin         GPIO_Pin_11
#define  BOX_CONTROL_GPIO           GPIOD


/*-------------------------------------------------------------------------------------------*/
/*           定时器宏定义                     */
#define RCC_APBxTimerClockCmd     RCC_APB1PeriphClockCmd
#define RCC_APBxPeriph_TIMx       RCC_APB1Periph_TIM3
#define TIMx                      TIM3
#define TIM_OCxInit               TIM_OC3Init
#define TIM_OCxPreloadConfig      TIM_OC3PreloadConfig
#define TIM_OCyInit               TIM_OC4Init
#define TIM_OCyPreloadConfig      TIM_OC4PreloadConfig
#define RCC_APBxGPIOxClockCmd     RCC_APB2PeriphClockCmd
/*           外部引脚宏定义                     */
#define TIMx_GPIO_Pin_IN1         GPIO_Pin_0    //对应的时TIM3的CH3
#define TIMx_GPIO_Pin_IN2         GPIO_Pin_1    //对应的时TIM3的CH4
#define TIMx_GPIOx                GPIOB
/*           电机限位开关引脚宏定义                     */
#define LIMIT_SWITCH_ON_INT_GPIO_PORT         GPIOE
#define LIMIT_SWITCH_ON_INT_GPIO_CLK          (RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO)
#define LIMIT_SWITCH_ON_INT_GPIO_PIN          GPIO_Pin_8
#define LIMIT_SWITCH_ON_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOE
#define LIMIT_SWITCH_ON_INT_EXTI_PINSOURCE    GPIO_PinSource8
#define LIMIT_SWITCH_ON_INT_EXTI_LINE         EXTI_Line8
#define LIMIT_SWITCH_ON_EXTI_IRQ              EXTI9_5_IRQn

#define LIMIT_SWITCH_OFF_INT_GPIO_PORT         GPIOE
#define LIMIT_SWITCH_OFF_INT_GPIO_CLK          (RCC_APB2Periph_GPIOE|RCC_APB2Periph_AFIO)
#define LIMIT_SWITCH_OFF_INT_GPIO_PIN          GPIO_Pin_9
#define LIMIT_SWITCH_OFF_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOE
#define LIMIT_SWITCH_OFF_INT_EXTI_PINSOURCE    GPIO_PinSource9
#define LIMIT_SWITCH_OFF_INT_EXTI_LINE         EXTI_Line9
#define LIMIT_SWITCH_OFF_EXTI_IRQ              EXTI9_5_IRQn

#define ANTI_COL_FW_BK_LIMIT_SWITCH_ON_OFF_IRQHandler            EXTI9_5_IRQHandler
#define IO_ON  1
#define IO_OFF 0


/*                                           */
#define            BOXDELAY_TIM                   TIM4
#define            BOXDELAY_TIM_APBxClock_FUN     RCC_APB1PeriphClockCmd
#define            BOXDELAY_TIM_CLK               RCC_APB1Periph_TIM4
#define            BOXDELAY_TIM_Period            (1000-1)
#define            BOXDELAY_TIM_Prescaler         71
#define            BOXDELAY_TIM_IRQ               TIM4_IRQn
#define            BOXDELAY_TIM_IRQHandler        TIM4_IRQHandler

/*---------------------外部变量声明-----------------------*/
extern volatile uint8_t  BoxReg;
#define BOXSTAREG 0x3
#define BOXSTA_ON       1
#define BOXSTA_OFF      0
#define BOXSTA_ING      2

#define BOXCTLREG 0xC


extern volatile uint16_t  BoxTimeOut;
/*---------------------外部函数声明-----------------------*/
void Box_Init(void);
uint8_t BoxControl(uint8_t cmd);
void Box_Motor_Stop(void);
void Box_LIMIT_SWITCH_ON_Process(void);
void Box_LIMIT_SWITCH_OFF_Process(void);
void Box_Motor_Forverd(int16_t Speed);
void Box_Motor_Reverse(int16_t Speed);
#endif   // __BSP_BOX_MOTOR_TIMER_H__
