#include "system_timer.h"
uint32_t time_1ms = 0, time_10ms,time_20ms = 0,time_50ms = 0,time_100ms = 0,time_200ms = 0, time_1000ms = 0 ,time_5000ms = 0 , time_30ms = 0,time_continue=0; // ms ��ʱ����
/*
 *@brief��ͨ�ö�ʱ��x��NVIC����
 *@param����
 *@retval����
 */
static void SYSTEM_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = SYSTEM_TIM_IRQ ;	
		// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	  // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *���ܣ�ͨ�ö�ʱ��2�ĵ�ģʽ����
 *������Frequency-����һ�ζ�ʱ���жϵ�Ƶ��
 *���أ���
 */
void SYSTEM_TIM_Init(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
    SYSTEM_TIM_APBxClock_FUN(SYSTEM_TIM_CLK, ENABLE);	
		// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // ʱ��Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler= SYSTEM_TIM_Prescaler;	
		// ʱ�ӷ�Ƶ���� ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// �ظ���������ֵ��û�õ����ù�
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // ��ʼ����ʱ��
    TIM_TimeBaseInit(SYSTEM_TIM, &TIM_TimeBaseStructure); 
		//NVIC������
		SYSTEM_TIM_NVIC_Config();
		// �����������ж�
    TIM_ITConfig(SYSTEM_TIM,TIM_IT_Update,ENABLE);
		// ʹ�ܼ�����
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

















