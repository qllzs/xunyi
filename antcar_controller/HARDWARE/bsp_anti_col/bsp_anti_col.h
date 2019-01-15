#ifndef __BSP_ANTI_COL_H__
#define __BSP_ANTI_COL_H__
#include "bsp_include.h"

/*-----------------宏定义--------------------------------*/
//防碰撞的引脚定义
#define ANTI_COL_FW_INT_GPIO_PORT         GPIOC
#define ANTI_COL_FW_INT_GPIO_CLK          (RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO)
#define ANTI_COL_FW_INT_GPIO_PIN          GPIO_Pin_5
#define ANTI_COL_FW_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOC
#define ANTI_COL_FW_INT_EXTI_PINSOURCE    GPIO_PinSource5
#define ANTI_COL_FW_INT_EXTI_LINE         EXTI_Line5
#define ANTI_COL_FW_INT_EXTI_IRQ          EXTI9_5_IRQn

#define ANTI_COL_BK_INT_GPIO_PORT         GPIOC
#define ANTI_COL_BK_INT_GPIO_CLK          (RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO)
#define ANTI_COL_BK_INT_GPIO_PIN          GPIO_Pin_4
#define ANTI_COL_BK_INT_EXTI_PORTSOURCE   GPIO_PortSourceGPIOC
#define ANTI_COL_BK_INT_EXTI_PINSOURCE    GPIO_PinSource4
#define ANTI_COL_BK_INT_EXTI_LINE         EXTI_Line4
#define ANTI_COL_BK_INT_EXTI_IRQ          EXTI4_IRQn
#define ANTI_COL_BK_IRQHandler            EXTI4_IRQHandler

/*-----------------外部函数声明--------------------------------*/
void AntiCol_EXTI_IO_Init(void);
void AntiColStatusCheck(int16_t *Vel , int16_t *Ang);
#endif  //__BSP_ANTI_COL_H__



