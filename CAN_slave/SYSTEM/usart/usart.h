#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
#define USART_REC_LEN  			100 
	  	  	
extern uint8_t  USART1_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节
  
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(uint32_t bound1);
#endif


