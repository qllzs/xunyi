#include "system_timer.h"
uint32_t time_1ms = 0, time_10ms,time_20ms = 0,time_50ms = 0,time_100ms = 0,time_200ms = 0, time_1000ms = 0 ,time_5000ms = 0 , time_30ms = 0,time_continue=0; // ms 计时变量
/*
 *@brief：通用定时器x的NVIC配置
 *@param：无
 *@retval：无
 */
static void SYSTEM_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = SYSTEM_TIM_IRQ ;	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	  // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *功能：通用定时器2的的模式配置
 *参数：Frequency-进入一次定时器中断的频率
 *返回：无
 */
void SYSTEM_TIM_Init(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// 开启定时器时钟,即内部时钟CK_INT=72M
    SYSTEM_TIM_APBxClock_FUN(SYSTEM_TIM_CLK, ENABLE);	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // 时钟预分频数
    TIM_TimeBaseStructure.TIM_Prescaler= SYSTEM_TIM_Prescaler;	
		// 时钟分频因子 ，没用到不用管
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// 计数器计数模式，设置为向上计数
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// 重复计数器的值，没用到不用管
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // 初始化定时器
    TIM_TimeBaseInit(SYSTEM_TIM, &TIM_TimeBaseStructure); 
		//NVIC的配置
		SYSTEM_TIM_NVIC_Config();
		// 开启计数器中断
    TIM_ITConfig(SYSTEM_TIM,TIM_IT_Update,ENABLE);
		// 使能计数器
    TIM_Cmd(SYSTEM_TIM, ENABLE);
}
/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
extern uint8_t time_CCD;
void  SYSTEM_TIM_IRQHandler (void)
{
//	#ifdef AUTOMATIC_EXPLORE
//		static uint8_t integration_point = 0; 
//	#endif
	if ( TIM_GetITStatus( SYSTEM_TIM, TIM_IT_Update) != RESET ) 
	{	
		time_1ms++;
		time_10ms++;
		time_20ms++;
		time_30ms++;
		time_50ms++;
		time_100ms++;
		time_200ms++;
		time_1000ms++;
		time_5000ms++;
		TimeStampReg++;
		time_continue++;
		time_CCD++;
		if(time_CCD >= 50)
			time_CCD = 0;
		
		TIM_ClearITPendingBit(SYSTEM_TIM , TIM_FLAG_Update);  		 
	}		 	
}

















