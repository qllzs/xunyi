#include "watchdog.h" 

/*-------------------Ӳ�����Ź�-------------------------*/
/*
 * ���� IWDG �ĳ�ʱʱ��
 * Tout = prv/40 * rlv (s)
 *      prv������[4,8,16,32,64,128,256]
 * prv:Ԥ��Ƶ��ֵ��ȡֵ���£�
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
 * rlv:Ԥ��Ƶ��ֵ��ȡֵ��ΧΪ��0-0XFFF
 * �������þ�����
 * IWDG_Config(IWDG_Prescaler_64 ,625);  // IWDG 1s ��ʱ���
 */

void IWDG_Config(uint8_t prv ,uint16_t rlv)
{	
	// ʹ�� Ԥ��Ƶ�Ĵ���PR����װ�ؼĴ���RLR��д
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
	
	// ����Ԥ��Ƶ��ֵ
	IWDG_SetPrescaler( prv );
	
	// ������װ�ؼĴ���ֵ
	IWDG_SetReload( rlv );
	
	// ����װ�ؼĴ�����ֵ�ŵ���������
	IWDG_ReloadCounter();
	
	// ʹ�� IWDG
	IWDG_Enable();	
}
// ι��
void IWDG_Feed(void)
{
	// ����װ�ؼĴ�����ֵ�ŵ��������У�ι������ֹIWDG��λ
	// ����������ֵ����0��ʱ������ϵͳ��λ
	IWDG_ReloadCounter();
}
/*                    ������Ź�                        */
//�Լ�д�Ŀ��Ź���ʱ�����飬1ms��һ�Σ���������������
uint16_t WatchDog[5] = {1000,1000,1000,1000,1000};
/*
 *@brief��ͨ�ö�ʱ��4��NVIC����
 *@param����
 *@retval����
 */
static void WDG_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = WDG_TIM_IRQ ;	
		// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	  // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *���ܣ�ͨ�ö�ʱ��2�ĵ�ģʽ����
 *������Frequency-����һ�ζ�ʱ���жϵ�Ƶ��
 *���أ���
 *@notice:APB1����ʱ��Ϊ36M���ǣ�������ʱ����ʱ����ͨ��CLKapb1*2=72M�õ���
 */
void WDG_TIM_Init(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=36M
    WDG_TIM_APBxClock_FUN(WDG_TIM_CLK, ENABLE);	
		// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // ʱ��Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler= WDG_TIM_Prescaler;	
		// ʱ�ӷ�Ƶ���� ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// �ظ���������ֵ��û�õ����ù�
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // ��ʼ����ʱ��
    TIM_TimeBaseInit(WDG_TIM, &TIM_TimeBaseStructure); 
		//NVIC������
		WDG_TIM_NVIC_Config();
		// �����������ж�
    TIM_ITConfig(WDG_TIM,TIM_IT_Update,ENABLE);
		// ʹ�ܼ�����
    TIM_Cmd(WDG_TIM, ENABLE);
		//���Ź���ʱ�������ʼ��	
}
/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void  WDG_TIM_IRQHandler (void)
{
	if ( TIM_GetITStatus( WDG_TIM, TIM_IT_Update) != RESET ) 
	{	
		WatchDog[PULSE_WATCHDOG]--;
		if(!WatchDog[PULSE_WATCHDOG])
		{
			WatchDog[PULSE_WATCHDOG] = PULSE_WDT_TIMEOUT;
			#ifndef BareDebug
			VelOfCar = 0;
			AngOfCar = 0; 
			Vel_Set = 0;
			Ang_Set = 0;
			ModeReg = 0;	
			FaultReg |=  PULSE_BIT;//�����ϼĴ�����ӦΪ��Ϊ
			#endif
		}
		TIM_ClearITPendingBit(WDG_TIM , TIM_FLAG_Update);  		 
	}		 	
}
