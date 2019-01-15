#include "bsp_anti_col.h"

/**
  * @brief  ����Ƕ�������жϿ�����NVIC
  * @param  ��
  * @retval ��
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* �����ж�Դ������ײ1 */
  NVIC_InitStructure.NVIC_IRQChannel = ANTI_COL_BK_INT_EXTI_IRQ;
  /* ������ռ���ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  /* ���������ȼ� */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* ʹ���ж�ͨ�� */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* �����ж�Դ������ײ2 ����ʹ������������� */  
  NVIC_InitStructure.NVIC_IRQChannel = ANTI_COL_FW_INT_EXTI_IRQ;
  NVIC_Init(&NVIC_InitStructure);	
}
 /**
  * @brief  ���� IOΪEXTI�жϿڣ��������ж����ȼ�
  * @param  ��
  * @retval ��
  */
void AntiCol_EXTI_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;

	/*��������GPIO�ڵ�ʱ��*/
	RCC_APB2PeriphClockCmd(ANTI_COL_BK_INT_GPIO_CLK,ENABLE);
	RCC_APB2PeriphClockCmd(ANTI_COL_FW_INT_GPIO_CLK,ENABLE);

	/* ���� NVIC �ж�*/
	NVIC_Configuration();
	
/*--------------------------����ײ����1����-----------------------------*/
	/* ѡ�񰴼��õ���GPIO */	
  GPIO_InitStructure.GPIO_Pin = ANTI_COL_BK_INT_GPIO_PIN;
  /* ����Ϊ�������� */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ANTI_COL_BK_INT_GPIO_PORT, &GPIO_InitStructure);

	/* ѡ��EXTI���ź�Դ */
  GPIO_EXTILineConfig(ANTI_COL_BK_INT_EXTI_PORTSOURCE, ANTI_COL_BK_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = ANTI_COL_BK_INT_EXTI_LINE;
	
	/* EXTIΪ�ж�ģʽ */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* ʹ���ж� */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
/*--------------------------����ײ����2����-----------------------------*/
	/* ѡ�񰴼��õ���GPIO */	
  GPIO_InitStructure.GPIO_Pin = ANTI_COL_FW_INT_GPIO_PIN;
  /* ����Ϊ�������� */	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(ANTI_COL_FW_INT_GPIO_PORT, &GPIO_InitStructure);

	/* ѡ��EXTI���ź�Դ */
  GPIO_EXTILineConfig(ANTI_COL_FW_INT_EXTI_PORTSOURCE, ANTI_COL_FW_INT_EXTI_PINSOURCE); 
  EXTI_InitStructure.EXTI_Line = ANTI_COL_FW_INT_EXTI_LINE;
	
	/* EXTIΪ�ж�ģʽ */
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/* �������ж� */
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  /* ʹ���ж� */	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
/**
 *@brief ��ͨ��������ײIO�ڵ�ƽ�����ٶȽ��п��ƣ�������ײ�������������Ĵ�����Ӧλ
 *@param ��������ٶ�
 *@retvel��none
 *@notice���˺���������С�����ٶ��趨��������
 */
void AntiColStatusCheck(int16_t *Vel , int16_t *Ang)
{
	if(GPIO_ReadInputDataBit(ANTI_COL_FW_INT_GPIO_PORT,ANTI_COL_FW_INT_GPIO_PIN) == IO_ON) // ǰ������
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
	if(GPIO_ReadInputDataBit(ANTI_COL_BK_INT_GPIO_PORT,ANTI_COL_BK_INT_GPIO_PIN) == IO_ON) // ��������
	{
		if(*Vel < 0)
		{
			*Vel = 0;
		  *Ang = 0;
		}
	}
	else //û�з���ǰ����ײ
	{
		FaultReg &= ~COL_BW_BIT;
	}
}
void ANTI_COL_BK_IRQHandler(void)
{
  //ȷ���Ƿ������EXTI Line�ж�
	if(EXTI_GetITStatus(ANTI_COL_BK_INT_EXTI_LINE) != RESET) 
	{
		FaultReg |= COL_BW_BIT;
		ModeReg = MOD_EMC_STOP;
    //����жϱ�־λ
		EXTI_ClearITPendingBit(ANTI_COL_BK_INT_EXTI_LINE);     
	} 
}





















