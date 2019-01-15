#include "bsp_anti_col.h"

/**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置中断源：防碰撞1 */
  NVIC_InitStructure.NVIC_IRQChannel = ANTI_COL_BK_INT_EXTI_IRQ;
  /* 配置抢占优先级 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* 配置子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* 使能中断通道 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* 配置中断源：防碰撞2 其他使用上面相关配置 */  
  NVIC_InitStructure.NVIC_IRQChannel = ANTI_COL_FW_INT_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);	
}
 /**
  * @brief  配置 IO为EXTI中断口，并设置中断优先级
  * @param  无
  * @retval 无
  */
void AntiCol_EXTI_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/*开启按键GPIO口的时钟*/
	RCC_APB2PeriphClockCmd(ANTI_COL_BK_INT_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(ANTI_COL_FW_INT_GPIO_CLK,ENABLE);

	/* 配置 NVIC 中断*/
	NVIC_Configuration();
	
/*--------------------------防碰撞开关1配置-----------------------------*/
	/* 选择按键用到的GPIO */	
  GPIO_InitStructure.GPIO_Pin = ANTI_COL_BK_INT_GPIO_PIN;
  /* 配置为浮空输入 */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ANTI_COL_BK_INT_GPIO_PORT, &GPIO_InitStructure);

	/* 选择EXTI的信号源 */
  GPIO_EXTILineConfig(ANTI_COL_BK_INT_EXTI_PORTSOURCE, ANTI_COL_BK_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = ANTI_COL_BK_INT_EXTI_LINE;
	
	/* EXTI为中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* 使能中断 */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
/*--------------------------防碰撞开关2配置-----------------------------*/
	/* 选择按键用到的GPIO */	
  GPIO_InitStructure.GPIO_Pin = ANTI_COL_FW_INT_GPIO_PIN;
  /* 配置为浮空输入 */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ANTI_COL_FW_INT_GPIO_PORT, &GPIO_InitStructure);

	/* 选择EXTI的信号源 */
  GPIO_EXTILineConfig(ANTI_COL_FW_INT_EXTI_PORTSOURCE, ANTI_COL_FW_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = ANTI_COL_FW_INT_EXTI_LINE;
	
	/* EXTI为中断模式 */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* 上升沿中断 */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* 使能中断 */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
/**
 *@brief ：通过检查防碰撞IO口电平来对速度进行控制，并在碰撞故障清除后清除寄存器相应位
 *@param ：传入的速度
 *@retvel：none
 *@notice：此函数将会在小车的速度设定函数调用
 */
void AntiColStatusCheck(int16_t *Vel , int16_t *Ang)
{
	if(GPIO_ReadInputDataBit(ANTI_COL_FW_INT_GPIO_PORT,ANTI_COL_FW_INT_GPIO_PIN) == IO_ON) // 前碰动作
	{
		if(*Vel > 0)
		{
			*Vel = 0;
		  *Ang = 0;
		}
	}
	else
	{
			FaultReg &= ~COL_FW_BIT;
	}
	if(GPIO_ReadInputDataBit(ANTI_COL_BK_INT_GPIO_PORT,ANTI_COL_BK_INT_GPIO_PIN) == IO_ON) // 后碰动作
	{
		if(*Vel < 0)
		{
			*Vel = 0;
		  *Ang = 0;
		}
	}
	else //没有发生前后碰撞
	{
		FaultReg &= ~COL_BW_BIT;
	}
}
void ANTI_COL_BK_IRQHandler(void)
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(ANTI_COL_BK_INT_EXTI_LINE) != RESET) 
	{
		FaultReg |= COL_BW_BIT;
		ModeReg = MOD_EMC_STOP;
    //清除中断标志位
		EXTI_ClearITPendingBit(ANTI_COL_BK_INT_EXTI_LINE);     
	} 
}





















