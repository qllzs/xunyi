#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
#define USART_REC_LEN  			100 
	  	  	
extern uint8_t  USART1_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�
  
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(uint32_t bound1);
#endif


