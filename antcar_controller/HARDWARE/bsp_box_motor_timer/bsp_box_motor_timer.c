#include "bsp_box_motor_timer.h"
//0-1λΪ����״̬�Ĵ�����0-Ϊ�ر�״̬ 1-Ϊ��״̬ 2-�Ƿ�̬����λ�������
volatile uint8_t   BoxReg = 0;
volatile uint16_t  BoxTimeOut = 1000;
#ifdef BOX_CONTROL_BY_UH
static void BoxControlIO_Init(void)
{
	GPIO_InitTypeDef          GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE); 

	//���õ��ʹ�õ��ùܽ�
	GPIO_InitStructure.GPIO_Pin = BOX_OPEN_GPIO_Pin | BOX_CLOSE_GPIO_Pin ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(BOX_CONTROL_GPIO, &GPIO_InitStructure);
}


#endif
/**
*@brief �������ŵ����ʹ�õ��Ķ�ʱ��x��ʼ������
*@param none
*@retval none
*@note PWMƵ�� = 72000000 / 4 / 1000 = 18Khz
*/
static void Timx_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			  TIM_OCInitStructure;
	GPIO_InitTypeDef          GPIO_InitStructure;
	
	RCC_APBxGPIOxClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
 	RCC_APBxTimerClockCmd(RCC_APBxPeriph_TIMx, ENABLE);

	//���õ��ʹ�õ��ùܽ�
	GPIO_InitStructure.GPIO_Pin = TIMx_GPIO_Pin_IN1 | TIMx_GPIO_Pin_IN2 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(TIMx_GPIOx, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //PWM��������	 
	TIM_TimeBaseStructure.TIM_Prescaler = 4 - 1; //����������ΪTIM2ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ��4��Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx���ϼ���ģʽ
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //����TIM_TimeBaseStructure��ָ���Ĳ�����ʼ������TIM3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0; //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	
	TIM_OCxInit(TIMx, &TIM_OCInitStructure);
	TIM_OCxPreloadConfig(TIMx, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OCyInit(TIMx, &TIM_OCInitStructure);
	TIM_OCyPreloadConfig(TIMx, TIM_OCPreload_Enable); //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ���
	
	TIM_ARRPreloadConfig(TIMx, ENABLE); //ʹ��TIM2��ARR�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIMx, ENABLE);  //ʹ��TIM3����
}
/**
  *@brief �������ת����
	*@param Speed-�趨���ٶȴ�С
	*@retval none
	*@note �ٶ���0-1000����ֵԽ���ʾ���Ŷ���Խ�죬�����ת�������ţ���ת�رճ���
	*/
void Box_Motor_Forverd(int16_t Speed)
{
	#ifdef BOX_CONTROL_BY_UH
	TIMx->CCR4 = 1000 - Speed;
	GPIO_SetBits(BOX_CONTROL_GPIO, BOX_OPEN_GPIO_Pin);
	GPIO_ResetBits(BOX_CONTROL_GPIO,BOX_CLOSE_GPIO_Pin);
	#else
	int16_t duty;
	duty = 1000 - Speed;
	TIMx->CCR3 = duty;
	TIMx->CCR4 = 1000;	
	#endif
	
	

}
/**
  *@brief ����ķ�ת����
	*@param Speed-�趨���ٶȴ�С
	*@retval none
	*@note �ٶ���0-1000����ֵԽ���ʾ���Ŷ���Խ�죬�����ת�������ţ���ת�رճ���
	*/
void Box_Motor_Reverse(int16_t Speed)
{

	#ifdef BOX_CONTROL_BY_UH
	TIMx->CCR4 = 1000 - Speed;
	GPIO_SetBits(BOX_CONTROL_GPIO, BOX_CLOSE_GPIO_Pin);
	GPIO_ResetBits(BOX_CONTROL_GPIO,BOX_OPEN_GPIO_Pin);
	#else
	int16_t duty;
	duty = 1000 - Speed;
	TIMx->CCR3 = 1000;
	TIMx->CCR4 = duty;	
	#endif
}
/**
	*@brief ���ŵ��ͣת
	*@param none
	*@retval none
	*/
void Box_Motor_Stop(void)
{

	#ifdef BOX_CONTROL_BY_UH
	GPIO_ResetBits(BOX_CONTROL_GPIO,BOX_OPEN_GPIO_Pin);
	GPIO_ResetBits(BOX_CONTROL_GPIO,BOX_CLOSE_GPIO_Pin);
	#else
	TIMx->CCR3 = 1000;
	TIMx->CCR4 = 1000;	
	#endif
}
/**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  /* �����ж�Դ��������λ����ON */
  NVIC_InitStructure.NVIC_IRQChannel = LIMIT_SWITCH_ON_EXTI_IRQ;
  /* ������ռ���ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* ���������ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* ʹ���ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/* �����ж�Դ��������λ����OFF ����ʹ������������� */ 
	NVIC_InitStructure.NVIC_IRQChannel = LIMIT_SWITCH_OFF_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);
	
}
/**
  * @brief  ���� IOΪEXTI�жϿڣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
static void Limit_Switch_EXTI_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(LIMIT_SWITCH_ON_INT_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(LIMIT_SWITCH_OFF_INT_GPIO_CLK,ENABLE);	
	/* ���� NVIC �ж�*/
	NVIC_Configuration();	
  /*--------------------------������λ����1������-----------------------------*/
	/* ѡ�񰴼��õ���GPIO */	
  GPIO_InitStructure.GPIO_Pin = LIMIT_SWITCH_ON_INT_GPIO_PIN;
  /* ����Ϊ�������� */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LIMIT_SWITCH_ON_INT_GPIO_PORT, &GPIO_InitStructure);

	/* ѡ��EXTI���ź�Դ */
  GPIO_EXTILineConfig(LIMIT_SWITCH_ON_INT_EXTI_PORTSOURCE, LIMIT_SWITCH_ON_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = LIMIT_SWITCH_ON_INT_EXTI_LINE;
	
	/* EXTIΪ�ж�ģʽ */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �½����ж� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* ʹ���ж� */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  /*--------------------------������λ����2������-----------------------------*/
	/* ѡ�񰴼��õ���GPIO */	
  GPIO_InitStructure.GPIO_Pin = LIMIT_SWITCH_OFF_INT_GPIO_PIN;
  /* ����Ϊ�������� */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LIMIT_SWITCH_OFF_INT_GPIO_PORT, &GPIO_InitStructure);

	/* ѡ��EXTI���ź�Դ */
  GPIO_EXTILineConfig(LIMIT_SWITCH_OFF_INT_EXTI_PORTSOURCE, LIMIT_SWITCH_OFF_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = LIMIT_SWITCH_OFF_INT_EXTI_LINE;
	
	/* EXTIΪ�ж�ģʽ */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* ʹ���ж� */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	
}

/*                                     ���ŵ����ͨ��պ���ʱ��ʱ����ʼ��                                      */
static void BoxDelay_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// �����ж���Դ
    NVIC_InitStructure.NVIC_IRQChannel = BOXDELAY_TIM_IRQ ;	
		// ���������ȼ�Ϊ 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	 
	  // ������ռ���ȼ�Ϊ3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *���ܣ����ŵ��������ʱ�õ��Ķ�ʱ����ģʽ����
 *������Frequency-����һ�ζ�ʱ���жϵ�Ƶ��
 *���أ���
 */
static void BOXDELAY_TIM_Mode_Config(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// ������ʱ��ʱ��,���ڲ�ʱ��CK_INT=72M
    BOXDELAY_TIM_APBxClock_FUN(BOXDELAY_TIM_CLK, ENABLE);	
		// �Զ���װ�ؼĴ�����ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»����ж�
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // ʱ��Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_Prescaler= BOXDELAY_TIM_Prescaler;	
		// ʱ�ӷ�Ƶ���� ��û�õ����ù�
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// ����������ģʽ������Ϊ���ϼ���
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// �ظ���������ֵ��û�õ����ù�
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // ��ʼ����ʱ��
    TIM_TimeBaseInit(BOXDELAY_TIM, &TIM_TimeBaseStructure);
		// ����������жϱ�־λ
    TIM_ClearFlag(BOXDELAY_TIM, TIM_FLAG_Update);
	  BoxDelay_TIM_NVIC_Config();
		// �����������ж�
    TIM_ITConfig(BOXDELAY_TIM,TIM_IT_Update,ENABLE);
		// ʹ�ܼ�����
    TIM_Cmd(BOXDELAY_TIM, DISABLE);
}

/**
	*@brief ���ų�ʼ������ʼ�����ŵĵ����������ŵ�״̬��������û�йر���ر�����
	*@param none
	*@retval none
	*/
void Box_Init(void)
{
	Timx_init();
	Limit_Switch_EXTI_IO_Config();
	BOXDELAY_TIM_Mode_Config(10000);//10ms����һ���ж�
	#ifdef BOX_CONTROL_BY_UH
	BoxControlIO_Init();
	#endif
	if(GPIO_ReadInputDataBit(LIMIT_SWITCH_OFF_INT_GPIO_PORT,LIMIT_SWITCH_OFF_INT_GPIO_PIN) != IO_ON)
	{
		BoxReg = BOXSTA_ING;
		/*********�����ת�������ţ���ת�رճ���************/
		TIM_Cmd(BOXDELAY_TIM, ENABLE);
		Box_Motor_Reverse(500);
	}
	else
	{
		BoxReg = BOXSTA_OFF;
	}
}
/**
	*@brief ���ſ��ƼĴ���
	*@param cmd ���ŵĿ���ָ��   1 ������        0 �ر�����
	*@retval 1 �ɹ�     0ʧ��
	*/
uint8_t BoxControl(uint8_t cmd)
{
	static uint8_t i_open = 0,i_close = 0;
	if(cmd)//������
	{
 		if((BoxReg & BOXSTAREG) != BOXSTA_ON)
		{
			TIM_Cmd(BOXDELAY_TIM, ENABLE);
			Box_Motor_Forverd(500);
			BoxReg = BOXSTA_ING;
			i_open++;
//			sprintf(WriteBuffer,"\n��ص�ѹΪ��%d",BatteryVoltage);
//			fileWrite(WriteBuffer,"0:��ص�ѹ����.txt");
			#ifndef Debug_CCD
			printf("��%d�δ�����\n\r",i_open);
			#endif
		}
		else
		{
			return 0;
		}
	}
	else//�ر�����
	{
		if((BoxReg & BOXSTAREG) != BOXSTA_OFF)
		{
			TIM_Cmd(BOXDELAY_TIM, ENABLE);
			Box_Motor_Reverse(500);
			BoxReg = BOXSTA_ING;
			i_close++;
			#ifndef Debug_CCD
			printf("��%d�ιر�����\n\r",i_close);
			#endif
		}
		else
		{
			return 0;
		}
	}
	return 1;
}
/**
  * @brief  This function handles BoxDelay interrupt request.
  * @param  None
  * @retval None
  */
void  BOXDELAY_TIM_IRQHandler (void)
{
	if ( TIM_GetITStatus( BOXDELAY_TIM, TIM_IT_Update) != RESET ) 
	{	
		if((BoxTimeOut--) < 10)
		{
			Box_Motor_Stop();
			BoxReg &= ~BOXSTAREG;//��ռĴ���
			BoxReg |= BOXSTA_ING;
			FaultReg |= BOX_BIT;
			BoxTimeOut = 1000;//��ʱ1000*10ms
			TIM_Cmd(BOXDELAY_TIM, DISABLE);
		}
		TIM_ClearITPendingBit(BOXDELAY_TIM , TIM_FLAG_Update);
	}		 	
}
void Box_LIMIT_SWITCH_OFF_Process(void)
{
		Box_Motor_Stop();
		BoxReg &= ~BOXSTAREG;//��ռĴ���
		BoxReg |= BOXSTA_OFF;
		FaultReg &= ~BOX_BIT;
		BoxTimeOut = 1000;//��ʱ1000*10ms
		TIM_Cmd(BOXDELAY_TIM, DISABLE);
}
/**
  * @brief  This function called by ANTI_COL_FW_BK_LIMIT_SWITCH_ON_IRQHandler interrupt.
  * @param  None
  * @retval None
  */
void Box_LIMIT_SWITCH_ON_Process(void)
{
		Box_Motor_Stop();
		BoxReg &= ~BOXSTAREG;//��ռĴ���
		BoxReg |= BOXSTA_ON;
		FaultReg &= ~BOX_BIT;
		BoxTimeOut = 1000;//��ʱ1000*10ms
		TIM_Cmd(BOXDELAY_TIM, DISABLE);
}



