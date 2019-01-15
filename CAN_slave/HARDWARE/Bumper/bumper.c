#include "bumper.h"
uint8_t	BumperReg=0;  //0x01:前置被触发   0x02:后置被触发;  0x03:前后置被触发
void UltrasonicUartInit(uint32_t bound1,uint32_t bound2)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//超声的时钟
	RCC_APB2PeriphClockCmd(UltrasonicCLK|UltrasonicGPIO_CLK, ENABLE);	
	//TX
	GPIO_InitStructure.GPIO_Pin			=	UltrasonicTX_PIN;
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed		=	GPIO_Speed_50MHz;
	GPIO_Init(UltrasonicGPIO_PORT,&GPIO_InitStructure);
	//RX
	GPIO_InitStructure.GPIO_Pin			=	UltrasonicRX_PIN;
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(UltrasonicGPIO_PORT,&GPIO_InitStructure);
	
	//中断配置
	NVIC_InitStructure.NVIC_IRQChannel	=	UltrasonicIRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel	=	USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_InitStructure.USART_BaudRate	=	bound1;
	USART_InitStructure.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode		=	USART_Mode_Rx;
	USART_InitStructure.USART_Parity	=	USART_Parity_No;
	USART_InitStructure.USART_StopBits	=	USART_StopBits_1;
	USART_InitStructure.USART_WordLength=	USART_WordLength_8b;
	USART_Init(UltrasonicUart,&USART_InitStructure);
	
	USART_InitStructure.USART_BaudRate	=	bound2;
	USART_InitStructure.USART_HardwareFlowControl	=	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode		=	USART_Mode_Rx|USART_Mode_Tx;
	USART_InitStructure.USART_Parity	=	USART_Parity_No;
	USART_InitStructure.USART_StopBits	=	USART_StopBits_1;
	USART_InitStructure.USART_WordLength=	USART_WordLength_8b;
	USART_Init(DebugUart,&USART_InitStructure);
		
	USART_ITConfig(UltrasonicUart,USART_IT_RXNE|USART_IT_IDLE,ENABLE);
	USART_ITConfig(DebugUart,USART_IT_RXNE,ENABLE);
	
	USART_Cmd(UltrasonicUart,ENABLE);
	USART_Cmd(DebugUart,ENABLE);
	
}

void BumperTriggerInit()
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	EXTI_InitTypeDef 	EXTI_InitStruct;
	//时钟
	RCC_APB2PeriphClockCmd(BUMPER_A_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(BUMPER_B_GPIO_CLK, ENABLE);
	//GPIO
	GPIO_InitStructure.GPIO_Pin			=	BUMPER_A_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(BUMPER_A_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin			=	BUMPER_B_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode		=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(BUMPER_B_GPIO_PORT,&GPIO_InitStructure);
	
	//exit
	GPIO_EXTILineConfig(BUMPER_A_EXTI_PORTSOURCE,BUMPER_A_EXTI_PINSOURCE);
	EXTI_InitStruct.EXTI_Line			=	BUMPER_A_EXTI_LINE;
	EXTI_InitStruct.EXTI_Mode			=	EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger		=	EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd		=	ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	GPIO_EXTILineConfig(BUMPER_B_EXTI_PORTSOURCE,BUMPER_B_EXTI_PINSOURCE);
	EXTI_InitStruct.EXTI_Line			=	BUMPER_B_EXTI_LINE;
	EXTI_InitStruct.EXTI_Mode			=	EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger		=	EXTI_Trigger_Rising_Falling;
	EXTI_InitStruct.EXTI_LineCmd		=	ENABLE;
	EXTI_Init(&EXTI_InitStruct);
	
	//nvic
	NVIC_InitStructure.NVIC_IRQChannel	=	BUMPER_A_EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel	=	BUMPER_B_EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void BumperInit()
{
	UltrasonicUartInit(9600,115200);
	BumperTriggerInit();
}

/***************
* BumperA 前置被触发
********************/
uint8_t hi=0;
void BUMPER_AIRQ_Handler()
{
	if(EXTI_GetITStatus(BUMPER_A_EXTI_LINE) != RESET)
	{
		if(GPIO_ReadInputDataBit(BUMPER_A_GPIO_PORT,BUMPER_A_GPIO_PIN) == 1)
		{
			BumperReg	|= FrontBumperTrig;
		}
		else
		{
			BumperReg	&= ~FrontBumperTrig;
		}
	}
	EXTI_ClearITPendingBit(BUMPER_A_EXTI_LINE);
}
/***************
*BumperB 后置被触发
********************/

void BUMPER_BIRQ_Handler()
{
	//GPIO_ReadInputDataBit
	if(EXTI_GetITStatus(BUMPER_B_EXTI_LINE) != RESET)
	{
		if(GPIO_ReadInputDataBit(BUMPER_B_GPIO_PORT,BUMPER_B_GPIO_PIN) == 1)
		{
			BumperReg	|=	BackBumperTrig;
		}
		else
		{
			BumperReg	&= ~BackBumperTrig;
		}
	}
	EXTI_ClearITPendingBit(BUMPER_B_EXTI_LINE);
}
uint16_t BumperUart_RX_STA=0;       //接收状态标记	
uint8_t BumperUart_RX_BUF[UltrasonicLen]={0x00};
uint8_t Ultra_RX_BUF[UltrasonicLen]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
uint16_t crc = 0;
void UltrasonicHandler()
{
	uint8_t Res;
	uint8_t i;//j;  //crc校验和
    static char start=0;
	crc = 0;
    if(USART_GetITStatus(UltrasonicUart, USART_IT_RXNE) != RESET)  //接收中断
    {			
        Res =USART_ReceiveData(UltrasonicUart); //读取接收到的数据
            
        if(Res == 0xAA) //如果接收到的第一位数据是0xAA
        {
			BumperUart_RX_STA = 0;     //数值索引清零
			start = 1;           //此变量用于判断包尾及校验和
        }
	
		if(start == 1)
		{
			BumperUart_RX_BUF[BumperUart_RX_STA] = Res ; //把接收到的数据放到数组里面来
			BumperUart_RX_STA++;//数值索引加一
						
			if((BumperUart_RX_STA >= UltrasonicLen)&&(BumperUart_RX_BUF[12]==0x55))
			{
			//计算校验和
				for(i=0;i<10;i++)
				{
					crc += BumperUart_RX_BUF[1+i];//crc=（CH1~CH8+备用位+备用位）的低八位
				}
									
				if((crc & 0xFF)  == BumperUart_RX_BUF[11])
				{		
					memcpy(Ultra_RX_BUF,BumperUart_RX_BUF,UltrasonicLen);												 				
				}			
				start = 0;
				BumperUart_RX_STA = 0;// 重新开始接收
				memset(BumperUart_RX_BUF,0,UltrasonicLen);
				crc=0;			
			}
			
		}
		USART_ClearITPendingBit(UltrasonicUart,USART_IT_RXNE);
	}	
}
