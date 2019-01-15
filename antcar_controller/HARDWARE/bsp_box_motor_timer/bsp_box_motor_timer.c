#include "bsp_box_motor_timer.h"
//0-1位为箱门状态寄存器：0-为关闭状态 1-为打开状态 2-非法态，高位含义待定
volatile uint8_t   BoxReg = 0;
volatile uint16_t  BoxTimeOut = 1000;
#ifdef BOX_CONTROL_BY_UH
static void BoxControlIO_Init(void)
{
	GPIO_InitTypeDef          GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE); 

	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = BOX_OPEN_GPIO_Pin | BOX_CLOSE_GPIO_Pin ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(BOX_CONTROL_GPIO, &GPIO_InitStructure);
}


#endif
/**
*@brief 控制箱门电机所使用到的定时器x初始化函数
*@param none
*@retval none
*@note PWM频率 = 72000000 / 4 / 1000 = 18Khz
*/
static void Timx_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			  TIM_OCInitStructure;
	GPIO_InitTypeDef          GPIO_InitStructure;
	
	RCC_APBxGPIOxClockCmd(RCC_APB2Periph_GPIOB , ENABLE); 
 	RCC_APBxTimerClockCmd(RCC_APBxPeriph_TIMx, ENABLE);

	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = TIMx_GPIO_Pin_IN1 | TIMx_GPIO_Pin_IN2 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(TIMx_GPIOx, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; //PWM计数上限	 
	TIM_TimeBaseStructure.TIM_Prescaler = 4 - 1; //设置用来作为TIM2时钟频率除数的预分频值，4分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIMx向上计数模式
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //根据TIM_TimeBaseStructure中指定的参数初始化外设TIM3
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0; //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	
	TIM_OCxInit(TIMx, &TIM_OCInitStructure);
	TIM_OCxPreloadConfig(TIMx, TIM_OCPreload_Enable); //使能TIM3在CCR3上的预装载寄存器
	TIM_OCyInit(TIMx, &TIM_OCInitStructure);
	TIM_OCyPreloadConfig(TIMx, TIM_OCPreload_Enable); //使能TIM3在CCR4上的预装载寄存器
	
	TIM_ARRPreloadConfig(TIMx, ENABLE); //使能TIM2在ARR上的预装载寄存器
	TIM_Cmd(TIMx, ENABLE);  //使能TIM3外设
}
/**
  *@brief 电机的正转函数
	*@param Speed-设定的速度大小
	*@retval none
	*@note 速度在0-1000，数值越大表示箱门动作越快，电机正转开启车门，翻转关闭车门
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
  *@brief 电机的反转函数
	*@param Speed-设定的速度大小
	*@retval none
	*@note 速度在0-1000，数值越大表示箱门动作越快，电机正转开启车门，翻转关闭车门
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
	*@brief 箱门电机停转
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
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;  
  /* 配置中断源：箱门限位开关ON */
  NVIC_InitStructure.NVIC_IRQChannel = LIMIT_SWITCH_ON_EXTI_IRQ;
  /* 配置抢占优先级 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* 配置子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	/* 配置中断源：箱门限位开关OFF 其他使用上面相关配置 */ 
	NVIC_InitStructure.NVIC_IRQChannel = LIMIT_SWITCH_OFF_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);
	
}
/**
  * @brief  配置 IO为EXTI中断口，并设置中断优先级
  * @param  无
  * @retval 无
  */
static void Limit_Switch_EXTI_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(LIMIT_SWITCH_ON_INT_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(LIMIT_SWITCH_OFF_INT_GPIO_CLK,ENABLE);	
	/* 配置 NVIC 中断*/
	NVIC_Configuration();	
  /*--------------------------箱门限位开关1的配置-----------------------------*/
	/* 选择按键用到的GPIO */	
  GPIO_InitStructure.GPIO_Pin = LIMIT_SWITCH_ON_INT_GPIO_PIN;
  /* 配置为浮空输入 */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LIMIT_SWITCH_ON_INT_GPIO_PORT, &GPIO_InitStructure);

	/* 选择EXTI的信号源 */
  GPIO_EXTILineConfig(LIMIT_SWITCH_ON_INT_EXTI_PORTSOURCE, LIMIT_SWITCH_ON_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = LIMIT_SWITCH_ON_INT_EXTI_LINE;
	
	/* EXTI为中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 下降沿中断 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* 使能中断 */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
  /*--------------------------箱门限位开关2的配置-----------------------------*/
	/* 选择按键用到的GPIO */	
  GPIO_InitStructure.GPIO_Pin = LIMIT_SWITCH_OFF_INT_GPIO_PIN;
  /* 配置为浮空输入 */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(LIMIT_SWITCH_OFF_INT_GPIO_PORT, &GPIO_InitStructure);

	/* 选择EXTI的信号源 */
  GPIO_EXTILineConfig(LIMIT_SWITCH_OFF_INT_EXTI_PORTSOURCE, LIMIT_SWITCH_OFF_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = LIMIT_SWITCH_OFF_INT_EXTI_LINE;
	
	/* EXTI为中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* 使能中断 */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	
}

/*                                     箱门电机开通与闭合延时定时器初始化                                      */
static void BoxDelay_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = BOXDELAY_TIM_IRQ ;	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;	 
	  // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *功能：箱门电机开启延时用到的定时器的模式配置
 *参数：Frequency-进入一次定时器中断的频率
 *返回：无
 */
static void BOXDELAY_TIM_Mode_Config(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// 开启定时器时钟,即内部时钟CK_INT=72M
    BOXDELAY_TIM_APBxClock_FUN(BOXDELAY_TIM_CLK, ENABLE);	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // 时钟预分频数
    TIM_TimeBaseStructure.TIM_Prescaler= BOXDELAY_TIM_Prescaler;	
		// 时钟分频因子 ，没用到不用管
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// 计数器计数模式，设置为向上计数
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// 重复计数器的值，没用到不用管
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // 初始化定时器
    TIM_TimeBaseInit(BOXDELAY_TIM, &TIM_TimeBaseStructure);
		// 清除计数器中断标志位
    TIM_ClearFlag(BOXDELAY_TIM, TIM_FLAG_Update);
	  BoxDelay_TIM_NVIC_Config();
		// 开启计数器中断
    TIM_ITConfig(BOXDELAY_TIM,TIM_IT_Update,ENABLE);
		// 使能计数器
    TIM_Cmd(BOXDELAY_TIM, DISABLE);
}

/**
	*@brief 箱门初始化，初始化箱门的电机并检查箱门的状态，若箱门没有关闭则关闭箱门
	*@param none
	*@retval none
	*/
void Box_Init(void)
{
	Timx_init();
	Limit_Switch_EXTI_IO_Config();
	BOXDELAY_TIM_Mode_Config(10000);//10ms产生一次中断
	#ifdef BOX_CONTROL_BY_UH
	BoxControlIO_Init();
	#endif
	if(GPIO_ReadInputDataBit(LIMIT_SWITCH_OFF_INT_GPIO_PORT,LIMIT_SWITCH_OFF_INT_GPIO_PIN) != IO_ON)
	{
		BoxReg = BOXSTA_ING;
		/*********电机正转开启车门，翻转关闭车门************/
		TIM_Cmd(BOXDELAY_TIM, ENABLE);
		Box_Motor_Reverse(500);
	}
	else
	{
		BoxReg = BOXSTA_OFF;
	}
}
/**
	*@brief 箱门控制寄存器
	*@param cmd 箱门的控制指令   1 打开箱门        0 关闭箱门
	*@retval 1 成功     0失败
	*/
uint8_t BoxControl(uint8_t cmd)
{
	static uint8_t i_open = 0,i_close = 0;
	if(cmd)//打开箱门
	{
 		if((BoxReg & BOXSTAREG) != BOXSTA_ON)
		{
			TIM_Cmd(BOXDELAY_TIM, ENABLE);
			Box_Motor_Forverd(500);
			BoxReg = BOXSTA_ING;
			i_open++;
//			sprintf(WriteBuffer,"\n电池电压为：%d",BatteryVoltage);
//			fileWrite(WriteBuffer,"0:电池电压数据.txt");
			#ifndef Debug_CCD
			printf("第%d次打开箱门\n\r",i_open);
			#endif
		}
		else
		{
			return 0;
		}
	}
	else//关闭箱门
	{
		if((BoxReg & BOXSTAREG) != BOXSTA_OFF)
		{
			TIM_Cmd(BOXDELAY_TIM, ENABLE);
			Box_Motor_Reverse(500);
			BoxReg = BOXSTA_ING;
			i_close++;
			#ifndef Debug_CCD
			printf("第%d次关闭箱门\n\r",i_close);
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
			BoxReg &= ~BOXSTAREG;//清空寄存器
			BoxReg |= BOXSTA_ING;
			FaultReg |= BOX_BIT;
			BoxTimeOut = 1000;//延时1000*10ms
			TIM_Cmd(BOXDELAY_TIM, DISABLE);
		}
		TIM_ClearITPendingBit(BOXDELAY_TIM , TIM_FLAG_Update);
	}		 	
}
void Box_LIMIT_SWITCH_OFF_Process(void)
{
		Box_Motor_Stop();
		BoxReg &= ~BOXSTAREG;//清空寄存器
		BoxReg |= BOXSTA_OFF;
		FaultReg &= ~BOX_BIT;
		BoxTimeOut = 1000;//延时1000*10ms
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
		BoxReg &= ~BOXSTAREG;//清空寄存器
		BoxReg |= BOXSTA_ON;
		FaultReg &= ~BOX_BIT;
		BoxTimeOut = 1000;//延时1000*10ms
		TIM_Cmd(BOXDELAY_TIM, DISABLE);
}



