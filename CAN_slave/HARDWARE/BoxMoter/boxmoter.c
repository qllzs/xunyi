#include "boxmoter.h"

uint8_t	BoxReg=0;
int		BoxDelayTime[BoxTimeInitCnt] =	{BoxTimeCnt};
/*********************************
*箱门控制接口 
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
*箱门的状态
************************************/
void  BoxStauts(uint8_t *Status)
{
	
}
/************************************
*PWM驱动初始化，PWM1:PA8;	PWM2:PA7;
*************************************/
void BoxTimePwmInit(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	GPIO_InitTypeDef          	GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_PWM2ClockCmd|RCC_PWM1ClockCmd|RCC_PWM2GPIOClockCmd|RCC_PWM1GPIOClockCmd|RCC_APB2Periph_AFIO,ENABLE);
	
	//GPIO口配置
	GPIO_InitStructure.GPIO_Pin 		= PWM1GPIOPIN; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(PWM1GPIOPORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 		= PWM2GPIOPIN ; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(PWM2GPIOPORT, &GPIO_InitStructure);
	
	//TIM配置
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
	//PWM1配置
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//当计数值小于设定值时输出低电平，故输入值越大转速越快
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Enable;//输出比较使能
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//装入比较器的值
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//起始低电平
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//空闲的时候输出低电平，TIM1有效
	TIM_OCInitStructure.TIM_OCNIdleState= 	TIM_OCIdleState_Reset;
	TIMPWMOC1Init(TIMPWM1,&TIM_OCInitStructure);
	
	//PWM2配置
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//当计数值小于设定值时输出低电平，故输入值越大转速越快
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Enable;//输出比较使能
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//装入比较器的值
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//起始低电平
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//空闲的时候输出低电平，TIM1有效
	TIM_OCInitStructure.TIM_OCNIdleState= 	TIM_OCIdleState_Reset;
	TIMPWMOC2Init(TIMPWM2,&TIM_OCInitStructure);
	
	TIM_PWM1OCPreloadConfig(TIMPWM1,TIM_OCPreload_Enable);//使能预装载
	TIM_PWM2OCPreloadConfig(TIMPWM2,TIM_OCPreload_Enable);//使能预装载
	
//	TIM_ARRPreloadConfig(TIMPWM1, ENABLE); //使能TIM1在ARR上的预装载寄存器
//	TIM_ARRPreloadConfig(TIMPWM2, ENABLE); //使能TIM3在ARR上的预装载寄存器
	
	TIM_Cmd(TIMPWM1, ENABLE);  //使能TIM1外设
	TIM_Cmd(TIMPWM2, ENABLE);  //使能TIM1外设
	
	TIM_CtrlPWMOutputs(TIMPWM1, ENABLE);
	TIM_CtrlPWMOutputs(TIMPWM2, ENABLE);
}
//PWM1接电机的红色，PWM2接电机的黑线
//需要正转，则需要PWM2的CCR一直为低电平
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
//箱门停止
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
	/////////////A的配置
	GPIO_InitStructure.GPIO_Pin		=	LIMIT_SWITCH_A_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIMIT_SWITCH_A_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(LIMIT_SWITCH_A_EXTI_PORTSOURCE,LIMIT_SWITCH_A_EXTI_PINSOURCE);
	
	EXTI_InitStructure.EXTI_Line	=	LIMIT_SWITCH_A_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger	=	EXTI_Trigger_Falling;//下降沿
	EXTI_InitStructure.EXTI_LineCmd	=	ENABLE;
	EXTI_StructInit(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel	=	LIMIT_SWITCH_A_EXTI_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority	=	0;
	NVIC_InitStructure.NVIC_IRQChannelCmd	=	ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/////////////B的配置
	GPIO_InitStructure.GPIO_Pin		=	LIMIT_SWITCH_B_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_IN_FLOATING;
	GPIO_Init(LIMIT_SWITCH_B_GPIO_PORT,&GPIO_InitStructure);
	
	GPIO_EXTILineConfig(LIMIT_SWITCH_B_EXTI_PORTSOURCE,LIMIT_SWITCH_B_EXTI_PINSOURCE);
	
	EXTI_InitStructure.EXTI_Line	=	LIMIT_SWITCH_B_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode	=	EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger	=	EXTI_Trigger_Falling;//下降沿
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
		// 不使能计数器
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
