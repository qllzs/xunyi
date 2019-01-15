#ifndef __BSP_REMOTE_H__
#define __BSP_REMOTE_H__
#include "bsp_include.h"
/******************硬件宏定义**********************/
#define Remote_RCC_APB2Periph_USARTx       RCC_APB2Periph_USART1
#define Remote_RCC_APB2Periph_GPIOx        RCC_APB2Periph_GPIOA
#define Remote_GPIO_Port                   GPIOA
#define Remote_GPIO_Pin_Tx                 GPIO_Pin_9
#define Remote_GPIO_Pin_Rx                 GPIO_Pin_10
#define Remote_USART_IRQn                  USART1_IRQn
#define Remote_USARTx                      USART1
#define Remote_USARTx_IRQHandler           USART1_IRQHandler

/******************变量声明**********************/
extern uint8_t Buff_Rx_Final[30];
extern uint8_t MessageRevFlag;
/******************函数声明**********************/

void bsp_remote_USART_Init(u32 bound);
int8_t SendData(uint8_t *SendBuff, uint8_t len);
/****************************************/

#endif
















