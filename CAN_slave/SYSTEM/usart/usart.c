#include "GlobalParam.h" 
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
extern u8 FinRecvFlag,BeginUpdate;
	  

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 0
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


//注意,读取USARTx->SR能避免莫名其妙的错误   	
uint8_t USART1_RX_BUF[USART_REC_LEN]={0};
uint8_t USART2_RX_BUF[USART_REC_LEN]={0};

void uart_init(uint32_t bound1)
{
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	/**************************波特率115200*******************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//USART3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.10
   
  //USART3_RX	  GPIOB.11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB.11  

  //Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound1;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;//接收模式

	USART_Init(USART3, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口1 
	/**********************配置完成************************************************/
	

}

uint8_t rx_statu=0;
uint8_t upmodule=0;
uint16_t FrameLen=0,rx_cnt=0;
void USART3_IRQHandler(void)
{
	u8 res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//接收到数据
	{	 
		res=USART_ReceiveData(USART3); 
		switch(rx_statu)
		{
			case 0:
				if(res == 0x55)
				{
					rx_statu++;
				}
			break;
			case 1:
				if(res == 0xAA)
				{
					rx_statu++;
				}
				else
				{
					rx_statu = 0;
				}
			break;
			case 2:
				if((res == 0x02) && (FinRecvFlag == 0))
				{
					rx_statu++;				
				}
				else
					rx_statu = 0;
			break;
			case 3:
				switch(res)
				{
					case 0x00:
						upmodule = 0;
						rx_statu++;
					break;
					case 0x01:
						upmodule = 1;
						rx_statu++;
					break;
					case 0x02:
						upmodule = 2;
						rx_statu++;
					break;
					default:
						upmodule = 0;
						rx_statu =0;
					break;
				}
			break;
			case 4:
				if(rx_cnt == 0)
				{
					FrameLen = (res & 0xff) << 8;
					rx_cnt++;
				}
				else if(rx_cnt == 1)
				{
					FrameLen |= (res & 0xff);
					if(rx_cnt < USART_REC_LEN)
					{
						rx_cnt=0;
						BeginUpdate = 1;
						rx_statu++;
					}
					else
					{
						rx_cnt=0;
						rx_statu=0;
					}
				}		
			break;
			case 5:
				USART1_RX_BUF[rx_cnt++] = res;
				if(rx_cnt >= FrameLen)
				{
					FinRecvFlag = 1;
					rx_statu = 0;
				}
			break;
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
} 




