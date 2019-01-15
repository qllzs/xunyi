#include "boxmoter.h"

uint8_t	BoxReg=0;
int		BoxDelayTime[BoxTimeInitCnt] =	{BoxTimeCnt};
/*********************************
*���ſ��ƽӿ� 
*************************************/
void BoxMotorControl(uint8_t Cmd)
{
	if(Cmd)
	{
		BoxMotorFwd(BoxMotorSpd);
	}
	else
	{
		BoxMotorRev(BoxMotorSpd);
	}
}
/********************************
*���ŵ�״̬
************************************/
void  BoxStauts(uint8_t *Status)
{
	
}
/************************************
*PWM������ʼ����PWM1:PA8;	PWM2:PA7;
*************************************/
void BoxTimePwmInit(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	GPIO_InitTypeDef          	GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_PWM2ClockCmd|RCC_PWM1ClockCmd|RCC_PWM2GPIOClockCmd|RCC_PWM1GPIOClockCmd|RCC_APB2Periph_AFIO,ENABLE);
	
	//GPIO������
	GPIO_InitStructure.GPIO_Pin 		= PWM1GPIOPIN; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(PWM1GPIOPORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 		= PWM2GPIOPIN ; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(PWM2GPIOPORT, &GPIO_InitStructure);
	
	//TIM����
	TIM_DeInit(TIMPWM1);
	TIM_TimeBaseStructure.TIM_Period	= 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler	= 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMPWM1,&TIM_TimeBaseStructure);
	
	TIM_DeInit(TIMPWM2);
	TIM_TimeBaseStructure.TIM_Period	= 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler	= 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIMPWM2,&TIM_TimeBaseStructure);
	//PWM1����
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//������ֵС���趨ֵʱ����͵�ƽ��������ֵԽ��ת��Խ��
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Enable;//����Ƚ�ʹ��
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//װ��Ƚ�����ֵ
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//��ʼ�͵�ƽ
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//���е�ʱ������͵�ƽ��TIM1��Ч
	TIM_OCInitStructure.TIM_OCNIdleState= 	TIM_OCIdleState_Reset;
	TIMPWMOC1Init(TIMPWM1,&TIM_OCInitStructure);
	
	//PWM2����
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//������ֵС���趨ֵʱ����͵�ƽ��������ֵԽ��ת��Խ��
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Enable;//����Ƚ�ʹ��
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//װ��Ƚ�����ֵ
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//��ʼ�͵�ƽ
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//���е�ʱ������͵�ƽ��TIM1��Ч
	TIM_OCInitStructure.TIM_OCNIdleState= 	TIM_OCIdleState_Reset;
	TIMPWMOC2Init(TIMPWM2,&TIM_OCInitStructure);
	
	TIM_PWM1OCPreloadConfig(TIMPWM1,TIM_OCPreload_Enable);//ʹ��Ԥװ��
	TIM_PWM2OCPreloadConfig(TIMPWM2,TIM_OCPreload_Enable);//ʹ��Ԥװ��
	
//	TIM_ARRPreloadConfig(TIMPWM1, ENABLE); //ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���
//	TIM_ARRPreloadConfig(TIMPWM2, ENABLE); //ʹ��TIM3��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIMPWM1, ENABLE);  //ʹ��TIM1����
	TIM_Cmd(TIMPWM2, ENABLE);  //ʹ��TIM1����
	
	TIM_CtrlPWMOutputs(TIMPWM1, ENABLE);
	TIM_CtrlPWMOutputs(TIMPWM2, ENABLE);
}
//PWM1�ӵ���ĺ�ɫ��PWM2�ӵ���ĺ���
//��Ҫ��ת������ҪPWM2��CCRһֱΪ�͵�ƽ
void BoxMotorFwd(uint16_t	Speed)
{
	TIM_Cmd(BoxTime, ENABLE);
	TIMPWM1->CCR2	= Speed;
	TIMPWM2->CCR3	= 1000;
}

void BoxMotorRev(uint16_t	Speed)
{
	TIM_Cmd(BoxTime, ENABLE);
	
	TIMPWM1->CCR2	= 1000;
	TIMPWM2->CCR3	= Speed;
}
//����ֹͣ
void BoxMotorStop(void)
{
	TIM_Cmd(BoxTime, DISABLE);
	BoxDelayTime[0] = BoxTimeCnt;
	TIMPWM1->CCR2	= 1000;
	TIMPWM2->CCR3	= 1000;
}
void BoxLimitSwitchConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|LIMIT_SWITCH_A_GPIO_CLK|LIMIT_SWITCH_B_GPIO_CLK,ENABLE);
	/////////////A������
	GPIO_InitStructure.GPIO_Pin		=	LIMIT_SWITCH_A_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIMIT_SWITCH_A_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(LIMIT_SWITCH_A_EXTI_PORTSOURCE,LIMIT_SWITCH_A_EXTI_PINSOURCE);
	
	EXTI_InitStructure.EXTI_Line	=	LIMIT_SWITCH_A_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger	=	EXTI_Trigger_Falling;//�½���
	EXTI_InitStructure.EXTI_LineCmd	=	ENABLE;
	EXTI_StructInit(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel	=	LIMIT_SWITCH_A_EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/////////////B������
	GPIO_InitStructure.GPIO_Pin		=	LIMIT_SWITCH_B_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIMIT_SWITCH_B_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(LIMIT_SWITCH_B_EXTI_PORTSOURCE,LIMIT_SWITCH_B_EXTI_PINSOURCE);
	
	EXTI_InitStructure.EXTI_Line	=	LIMIT_SWITCH_B_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger	=	EXTI_Trigger_Falling;//�½���
	EXTI_InitStructure.EXTI_LineCmd	=	ENABLE;
	EXTI_StructInit(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel	=	LIMIT_SWITCH_B_EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void BoxMotorInit(void)
{
	BoxTimePwmInit();
	BoxLimitSwitchConfig();
	BoxTimeInit();

	if( GPIO_ReadInputDataBit(LIMIT_SWITCH_A_GPIO_PORT, LIMIT_SWITCH_A_GPIO_PIN) != 0)
	{
		BoxReg	= StaBoxIng;
		BoxMotorRev(BoxMotorSpd);
		//BoxMotorFwd(BoxMotorSpd);
	}
	
}


void LIMIT_SWITCH_AIRQ_Handler()
{
	if(EXTI_GetITStatus(LIMIT_SWITCH_A_EXTI_LINE) != RESET)
	{
		if(LIMIT_SWITCH_AIRQ_READ() == 0)
		{
			BoxMotorStop();
			BoxReg	= CmdBoxOff;
		}
		EXTI_ClearITPendingBit(LIMIT_SWITCH_A_EXTI_LINE);
	}
}

void LIMIT_SWITCH_BIRQ_Handler()
{
	if(EXTI_GetITStatus(LIMIT_SWITCH_B_EXTI_LINE) != RESET)
	{
		if(LIMIT_SWITCH_BIRQ_READ() == 0)
		{
			BoxMotorStop();
			BoxReg	= CmdBoxOn;
		}
		EXTI_ClearITPendingBit(LIMIT_SWITCH_B_EXTI_LINE);
	}
}



void BoxTimeInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	
	NVIC_InitTypeDef 		 NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler= Box_TIM_Prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseStructure.TIM_Period= BoxDelayFreq -1;	
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
	TIM_TimeBaseStructure.TIM_RepetitionCounter  = 0;
	
	TIM_TimeBaseInit(BoxTime, &TIM_TimeBaseStructure); 
	TIM_ClearFlag(BoxTime, TIM_FLAG_Update);
	NVIC_InitStructure.NVIC_IRQChannel = BoxTimeIRQ ;	
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(BoxTime,TIM_IT_Update,ENABLE);
		// ��ʹ�ܼ�����
	TIM_Cmd(BoxTime, DISABLE);
}

void BoxTimeHandler()
{
	if((BoxDelayTime[0]--) < 10)
	{
		BoxMotorStop();
		BoxDelayTime[0] = BoxTimeCnt;
		BoxReg	= StaBoxIng;
	}
	TIM_ClearITPendingBit(BoxTime , TIM_FLAG_Update);
}
