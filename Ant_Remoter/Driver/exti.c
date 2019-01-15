#include "board.h"
 
extern int channel;  //默认40通道传输

//外部中断初始化函数
void EXTIX_Init(void)
{
 
 	  EXTI_InitTypeDef EXTI_InitStructure;
 	  NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO ,ENABLE);//外部中断，需要使能AFIO时钟

	  KEY_Init();    //初始化按键对应io模式

    //GPIOB.12中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource12);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;//下降沿触发
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	 	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器

    //GPIOB.14	  中断线以及中断初始化配置
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	  	//根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
 
  	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//使能按键所在的外部中断通道
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	//抢占优先级2 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;					//子优先级1
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
  	NVIC_Init(&NVIC_InitStructure);  	  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
}

void EXTI15_10_IRQHandler(void)
{
  delay_ms(10);    //消抖
  if(KEY0 == 0)  //左侧按键
			{
				channel -= 20;  
				if(channel <= 0)
				{
					channel = 0;
				}
				NRF_TX_MODE(&channel);//初始化为发送模式
			}
 	EXTI_ClearITPendingBit(EXTI_Line12);    //清除LINE12上的中断标志位 
			
  if(KEY1 == 0) //右侧按键
			{
				channel += 20;
				if(channel >= 80)
				{
					channel = 80;
				}
			  NRF_TX_MODE(&channel);//初始化为发送模式
			}
	 EXTI_ClearITPendingBit(EXTI_Line14);  //清除LINE14线路挂起位
}
