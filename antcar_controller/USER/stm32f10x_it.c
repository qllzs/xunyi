/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_include.h"
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}
/**
	*@brief 前后防碰撞传感器IO及开箱门处限位开关的中断处理函数
	*@param none
	*@retval none
	*/
void ANTI_COL_FW_BK_LIMIT_SWITCH_ON_OFF_IRQHandler(void)
{
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(ANTI_COL_FW_INT_EXTI_LINE) != RESET) 
	{
		FaultReg |= COL_FW_BIT;
		ModeReg = MOD_EMC_STOP;
    //清除中断标志位
		EXTI_ClearITPendingBit(ANTI_COL_FW_INT_EXTI_LINE);     
	} 
  //确保是否产生了EXTI Line中断
	if(EXTI_GetITStatus(LIMIT_SWITCH_ON_INT_EXTI_LINE) != RESET) 
	{
		Box_LIMIT_SWITCH_ON_Process();
    //清除中断标志位
		EXTI_ClearITPendingBit(LIMIT_SWITCH_ON_INT_EXTI_LINE);     
	} 
	if(EXTI_GetITStatus(LIMIT_SWITCH_OFF_INT_EXTI_LINE) != RESET) 
	{
		Box_LIMIT_SWITCH_OFF_Process();
    //清除中断标志位
		EXTI_ClearITPendingBit(LIMIT_SWITCH_OFF_INT_EXTI_LINE);     
	} 	
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
