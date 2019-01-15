#include "GlobalParam.h"

uint32_t time_100ms = 0;

void SYSTEM_TIM_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn ;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}


void System_TimeInit(uint16_t Freq)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler= SYSTEM_TIM_Prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseStructure.TIM_Period= Freq - 1;	
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
	TIM_TimeBaseStructure.TIM_RepetitionCounter  = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
	
	SYSTEM_TIM_NVIC_Config();
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
		// 使能计数器
  TIM_Cmd(TIM2, ENABLE);
}


void TIM2_IRQHandler(void)
{
	if ( TIM_GetITStatus( TIM2, TIM_IT_Update) != RESET ) 
	{
		time_100ms++;
		
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);
	}
	
}
