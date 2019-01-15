#include "FaultProcess.h"
//故障寄存器定义
volatile uint16_t FaultReg = 0;
/*
 *功能：检查各种故障并通过LED显示故障状态，向上层发送故障状态
 *参数：FaultReg故障寄存器的值
 *返回：1-有故障 0-无故障
 *说明：LED灯的使用只在此函数调用，其他函数禁止使用LED灯（除初始化外）
 */
void FaultProcess(uint16_t Fault)
{	
	static uint16_t err_cnt=0;
	#ifdef METEC_MOTOR
	if(Fault & MOTOR_BIT)
	{
		LED_RED;  //红灯
		SendAlarmMessageToHost(TimeStampReg,MOTOR_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	#else
	if(MotorFault)
	{
		LED_RED;  //红灯
		SendAlarmMessageToHost(TimeStampReg,MOTOR_ID, ALARM_LEVEL_EMC,0);
		ClearFault_EnableAgain();
	}
	#endif
	if(Fault & BATTERY_BIT)
	{
		
		
		if( BatteryRemainder < 20)
		{
			LED_WHITE; //白灯
			SendAlarmMessageToHost(TimeStampReg,BATTERY_ID, ALARM_LEVEL_EMC,0);
		}
		else
		{
			LED_YELLOW;//黄灯
			
			SendAlarmMessageToHost(TimeStampReg,BATTERY_ID, ALARM_LEVEL_WARNING,0);
		}
		//delay_ms(FAULTDLY);
	}
	if(Fault & COL_FW_BIT)
	{
		LED_BLUE;//蓝色
		SendAlarmMessageToHost(TimeStampReg,COLLISION_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	if(Fault & COL_BW_BIT)
	{
		LED_BLUE;//蓝色
		SendAlarmMessageToHost(TimeStampReg,COLLISION_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	
	if(Fault & BOX_BIT)
	{
		LED_PURPLE;//紫色
		SendAlarmMessageToHost(TimeStampReg,BOX_ID, ALARM_LEVEL_EMC,0);
		//delay_ms(FAULTDLY);
	}
	
	if(Fault & PULSE_BIT)
	{
		LED_RGBOFF;//黑色
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
		LED_GREEN;//没有任何故障则显示绿灯
	}

}














