#include "FaultProcess.h"
//���ϼĴ�������
volatile uint16_t FaultReg = 0;
/*
 *���ܣ������ֹ��ϲ�ͨ��LED��ʾ����״̬�����ϲ㷢�͹���״̬
 *������FaultReg���ϼĴ�����ֵ
 *���أ�1-�й��� 0-�޹���
 *˵����LED�Ƶ�ʹ��ֻ�ڴ˺������ã�����������ֹʹ��LED�ƣ�����ʼ���⣩
 */
void FaultProcess(uint16_t Fault)
{	
	static uint16_t err_cnt=0;
	#ifdef METEC_MOTOR
	if(Fault & MOTOR_BIT)
	{
		LED_RED;  //���
		SendAlarmMessageToHost(TimeStampReg,MOTOR_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	#else
	if(MotorFault)
	{
		LED_RED;  //���
		SendAlarmMessageToHost(TimeStampReg,MOTOR_ID, ALARM_LEVEL_EMC,0);
		ClearFault_EnableAgain();
	}
	#endif
	if(Fault & BATTERY_BIT)
	{
		
		
		if( BatteryRemainder < 20)
		{
			LED_WHITE; //�׵�
			SendAlarmMessageToHost(TimeStampReg,BATTERY_ID, ALARM_LEVEL_EMC,0);
		}
		else
		{
			LED_YELLOW;//�Ƶ�
			
			SendAlarmMessageToHost(TimeStampReg,BATTERY_ID, ALARM_LEVEL_WARNING,0);
		}
		//delay_ms(FAULTDLY);
	}
	if(Fault & COL_FW_BIT)
	{
		LED_BLUE;//��ɫ
		SendAlarmMessageToHost(TimeStampReg,COLLISION_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	if(Fault & COL_BW_BIT)
	{
		LED_BLUE;//��ɫ
		SendAlarmMessageToHost(TimeStampReg,COLLISION_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	
	if(Fault & BOX_BIT)
	{
		LED_PURPLE;//��ɫ
		SendAlarmMessageToHost(TimeStampReg,BOX_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	
	if(Fault & PULSE_BIT)
	{
		LED_RGBOFF;//��ɫ
	//	SendAlarmMessageToHost(uint32_t time_stamp,uint32_t alarm_ID, uint8_t alarm_level,uint32_t alarm_paral);
		//delay_ms(FAULTDLY);	
	}
	else
	{
		LED_GREEN;
	}
	
	if(Fault)
		if(err_cnt++ >100)
			err_cnt=0;
		
	if( (err_cnt!=0) || (Fault) )
	{
		err_cnt--;
		return;
	}
	if(!Fault)
	{
		LED_GREEN;//û���κι�������ʾ�̵�
	}

}














