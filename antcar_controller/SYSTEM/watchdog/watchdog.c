#include "watchdog.h" 

/*-------------------硬件看门狗-------------------------*/
/*
 * 设置 IWDG 的超时时间
 * Tout = prv/40 * rlv (s)
 *      prv可以是[4,8,16,32,64,128,256]
 * prv:预分频器值，取值如下：
 *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
 *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
 *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
 *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
 *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
 *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
 *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
 *
 * rlv:预分频器值，取值范围为：0-0XFFF
 * 函数调用举例：
 * IWDG_Config(IWDG_Prescaler_64 ,625);  // IWDG 1s 超时溢出
 */

void IWDG_Config(uint8_t prv ,uint16_t rlv)
{	
	// 使能 预分频寄存器PR和重装载寄存器RLR可写
	IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
	
	// 设置预分频器值
	IWDG_SetPrescaler( prv );
	
	// 设置重装载寄存器值
	IWDG_SetReload( rlv );
	
	// 把重装载寄存器的值放到计数器中
	IWDG_ReloadCounter();
	
	// 使能 IWDG
	IWDG_Enable();	
}
// 喂狗
void IWDG_Feed(void)
{
	// 把重装载寄存器的值放到计数器中，喂狗，防止IWDG复位
	// 当计数器的值减到0的时候会产生系统复位
	IWDG_ReloadCounter();
}
/*                    软件看门狗                        */
//自己写的看门狗定时器数组，1ms减一次，减到零则处理事情
uint16_t WatchDog[5] = {1000,1000,1000,1000,1000};
/*
 *@brief：通用定时器4的NVIC配置
 *@param：无
 *@retval：无
 */
static void WDG_TIM_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 	
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = WDG_TIM_IRQ ;	
		// 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	  // 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/*
 *功能：通用定时器2的的模式配置
 *参数：Frequency-进入一次定时器中断的频率
 *返回：无
 *@notice:APB1总线时钟为36M但是，各个定时器的时钟是通过CLKapb1*2=72M得到的
 */
void WDG_TIM_Init(uint16_t Frequency)
{
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;		
		// 开启定时器时钟,即内部时钟CK_INT=36M
    WDG_TIM_APBxClock_FUN(WDG_TIM_CLK, ENABLE);	
		// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Period= Frequency - 1;
	  // 时钟预分频数
    TIM_TimeBaseStructure.TIM_Prescaler= WDG_TIM_Prescaler;	
		// 时钟分频因子 ，没用到不用管
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;		
		// 计数器计数模式，设置为向上计数
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		
		// 重复计数器的值，没用到不用管
		TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	  // 初始化定时器
    TIM_TimeBaseInit(WDG_TIM, &TIM_TimeBaseStructure); 
		//NVIC的配置
		WDG_TIM_NVIC_Config();
		// 开启计数器中断
    TIM_ITConfig(WDG_TIM,TIM_IT_Update,ENABLE);
		// 使能计数器
    TIM_Cmd(WDG_TIM, ENABLE);
		//看门狗定时器数组初始化	
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
			FaultReg |=  PULSE_BIT;//将故障寄存器相应为置为
			#endif
		}
		TIM_ClearITPendingBit(WDG_TIM , TIM_FLAG_Update);  		 
	}		 	
}
