#include "protocol.h"
/***********************ȫ�ֱ�������*********************************/
uint8_t InstructionDoOverFlag = 1;//ָ��ִ����ϱ�־��1��ʾִ����� 0��ʾδִ�����
uint8_t LineWidtherTime=0;
volatile uint32_t CarID = CAR_ID;
volatile uint32_t TimeStampReg = 0;
volatile uint8_t  ModeReg = 0;//����ģʽ���ƼĴ�����0=�ϲ���ƣ� 1=Ѳ��ģʽ��2=  Ѳ�ߺ���;3=��ͣ
volatile uint8_t  _Line_Sta = 0;//Ѱ�߼��״̬1=��⵽�ߣ�0=û�м�⵽��
volatile uint8_t  LineGraduChge = 0; //���𽥱仯 0û�о��� 1:���� ;��ģʽ�л�������
volatile uint8_t UpDownChangeMode =0 ; //0:��Ѳ���е��ϲ���ơ�1�����ϲ�����е�Ѳ��ģʽ�����û�д�Ѳ���е��ϲ������һֱΪ1
//�������ݶ���
volatile uint32_t Time_Stamp_Set = 0;
volatile int16_t Vel_Set = 0;
volatile int16_t Ang_Set = 0;
volatile struct Instruction_Struct Instruction_Structure;
volatile struct ParaConfig_Struct ParaConfigStructure;
//�������ݽṹ�����ݶ���
struct Pulse_Up_Struct Pulse_Up_Structure;
struct SpeedFeedBack_Struct SpeedFeedBack_Structure;
struct Alarm_struct Alarm_structure;
struct InstructionFeedBack_Struct InstructionFeedBack_Structure;
struct PowerConsume_Struct PowerConsume_Structure;
/*
 *���ܣ��õ�У���
 *������data-���������ָ�룬len-���ݳ���
 *���أ�check_sum ����У���
 */
static uint16_t GetCheckSum(uint8_t* Date,uint8_t len)
{
	uint8_t i=0;
	uint16_t CheckSum = 0;
	for(i=0;i<len;i++)
	{
		CheckSum += Date[(i + 4)]; 
	}
	return CheckSum;
}
/*���ܣ����յ������ݽ��н���
*����: Buff_Rx_Final:�յ����ݵ�ָ�� MessageRevFlag:���ݽ��ձ�־λ
*���أ�1:�����ɹ���-1:����ʧ�ܣ�
 */
int8_t ReceiveDataAnalysis(uint8_t DataBuff[30])
{
	uint8_t msg_id = DataBuff[5];	
	switch (msg_id)//�ж���ϢID
	{
		case PULSE_DOWN:
		{
			Time_Stamp_Set = (uint32_t)( DataBuff[6] | (DataBuff[7] << 8) | (DataBuff[8] << 16) | (DataBuff[9] << 24) );
			WatchDog[PULSE_WATCHDOG] = PULSE_WDT_TIMEOUT;
			FaultReg &=  ~PULSE_BIT;//�����ϼĴ�����Ӧλ����
			break;
		}	
		case SPEED_CONFIG:
		{
			Vel_Set = (int16_t)( DataBuff[6] | (DataBuff[7] << 8) );
			Ang_Set = (int16_t)( DataBuff[8] | (DataBuff[9] << 8) );
			break;
		}	
		case INSTRUCTION:
		{
			Instruction_Structure.cmd = (uint32_t)( DataBuff[6] | (DataBuff[7] << 8) | (DataBuff[8] << 16) | (DataBuff[9] << 24) );
			Instruction_Structure.para1 = DataBuff[10];
			Instruction_Structure.para2 = DataBuff[11];
			Instruction_Structure.para3 = DataBuff[12];
			Instruction_Structure.para4 = DataBuff[13];
			InstructionDoOverFlag = 1; 
			break;		
		}

		case PARAMETER_CONFIG:
		{
			ParaConfigStructure.para_id = (uint32_t)( DataBuff[6] | (DataBuff[7] << 8) | (DataBuff[8] << 16) | (DataBuff[9] << 24) );
			ParaConfigStructure.value1 = DataBuff[10];
			ParaConfigStructure.value2 = DataBuff[11];
			ParaConfigStructure.value3 = DataBuff[12];
			ParaConfigStructure.value4 = DataBuff[13];
			//ParaConfigStructure.check_sum = check_sum_rcv;
			switch ( ParaConfigStructure.para_id )
  			{
//				case CFG_ANTCAR_ID:
//				{
//					CarID = (ParaConfigStructure.value1 | (ParaConfigStructure.value2 << 8) |(ParaConfigStructure.value3 << 16) |(ParaConfigStructure.value4 << 24));
//					break;
//				}
//				case CFG_ALARM:
//				{
//					warn_v = (float)(ParaConfigStructure.value1 | (ParaConfigStructure.value2 << 8))*0.01;
//					break;
//				}
//				case CFG_MAX_SPEED:
//				{
//					max_speed = (float)(ParaConfigStructure.value1 | (ParaConfigStructure.value2 << 8));
//					break;
//				}
//				default:
//				{
//					break;
//				}
  			}
			break;
		}
		default:
			return -1;//У��ʧ��
	}
	return 1;
}
/*
*@brief:ָ�����
*@param:Instruction --ָ��ṹ��
*@ret:����1-ִ�����
*/
uint8_t Do_Instruction(volatile struct Instruction_Struct Instruction)
{
			switch (Instruction.cmd)
			{
				case CMD_STOP_EMC:
				{
					Vel_Set = 0;
					Ang_Set = 0;
					Pulse_Up_Structure.mode = MOD_EMC_STOP;
					break;
				}
				case CMD_MODE_CTL:
				{
					if(CMD_MODE_CTL_CONTROLER == Instruction.para1)
					{
							ModeReg = MOD_UP_CONTROL;
							UpDownChangeMode = 0;
					}
					else if(CMD_MODE_CTL_LINE_FOLLOW == Instruction.para1)
					{
							ModeReg = MOD_LINE_FOLLOW;
//							UpDownChangeMode = 1;
					}
					else if((CMD_MODE_CTL_LINE_BACKFOLLOW == Instruction.para1))
					{
							ModeReg = MOD_LINE_BACKFOLLOW;
					}
//					memset(WriteBuffer,0,sizeof(WriteBuffer));
//					sprintf(WriteBuffer,"\n�ϲ����ָ�M=%d",Instruction.para1);
//					fileWrite(WriteBuffer,"0:�յ��ϲ����ָ��.txt");
					break;
				}
				case CMD_BOX_STATUS:
				{
					
					if(Instruction.para1 == CMD_BOX_STATUS_CTL)
					{
						BoxControl(Instruction.para2);
					}
					else//��ѯ����״ָ̬��
					{

					}
					break;
				}
				case CMD_LINE_STATUS: //���ù�
				{
					break;
				}
				default:
				{
					break;
				}
			}
			return 0;
}

int8_t SendPulseMessageToHost(uint32_t time_stamp,uint8_t CarMode, uint8_t BoxStatus,uint8_t Car_status)
{
	uint8_t sendbuff[30] = {0},i = 0;
	uint16_t CheckSum = 0;
	//��С��ID���и�ֵ��buffer
	sendbuff[i++] = (uint8_t)( (CarID >> 24) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 16) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 8 ) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 0 ) & 0xff);
	sendbuff[i++] = 9;
	sendbuff[i++] = PULSE_UP;
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (2*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (3*8)));
	sendbuff[i++] = CarMode;
	sendbuff[i++] = BoxStatus;
	sendbuff[i++] = Car_status;
	CheckSum = GetCheckSum(sendbuff,9);
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (1*8)));
	return SendData(sendbuff, i);
}

int8_t SendSpeedMessageToHost(uint16_t velocity , uint16_t angular)
{
	uint8_t sendbuff[30] = {0},i = 0;
	uint16_t CheckSum = 0;
	sendbuff[i++] = (uint8_t)( (CarID >> 24) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 16) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 8 ) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 0 ) & 0xff);
	sendbuff[i++] = 6;
	sendbuff[i++] = SPEED_FEEDBACK;
	sendbuff[i++] = (uint8_t)( 0xff & (velocity >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (velocity >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (angular >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (angular >> (1*8)));
	CheckSum = GetCheckSum(sendbuff,6);
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (1*8)));
	return SendData(sendbuff, i);
}
int8_t SendAlarmMessageToHost(uint32_t time_stamp,uint32_t alarm_ID, uint8_t alarm_level,uint32_t alarm_paral)
{
	uint8_t sendbuff[30] = {0},i = 0;
	uint16_t CheckSum = 0;
	sendbuff[i++] = (uint8_t)( (CarID >> 24) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 16) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 8 ) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 0 ) & 0xff);
	sendbuff[i++] = 15;
	sendbuff[i++] = ALARM;
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (2*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_stamp >> (3*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_ID >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_ID >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_ID >> (2*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_ID >> (3*8)));
	sendbuff[i++] = alarm_level;
			
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_paral >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_paral >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_paral >> (2*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (alarm_paral >> (3*8)));
	CheckSum = GetCheckSum(sendbuff,6);
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (1*8)));
	return SendData(sendbuff, i);
}

int8_t SendInstructionFeedbackMessageToHost(uint8_t status,uint8_t degree)
{
	uint8_t sendbuff[30] = {0},i = 0;
	uint16_t CheckSum = 0;
	sendbuff[i++] = (uint8_t)( (CarID >> 24) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 16) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 8 ) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 0 ) & 0xff);
	sendbuff[i++] = 4;
	sendbuff[i++] = INSTRUCTION_FEEDBACK;
	sendbuff[i++] = status;
	sendbuff[i++] = degree;
	CheckSum = GetCheckSum(sendbuff,4);
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (1*8)));
	return SendData(sendbuff, i);
}
int8_t  SendVoltageMessageToHost(uint32_t time_std_s,uint16_t voltages,uint16_t current_battery,uint8_t battery_remaining,uint16_t time_can_run)
{
	uint8_t sendbuff[30] = {0},i = 0;
	uint16_t CheckSum = 0;
	sendbuff[i++] = (uint8_t)( (CarID >> 24) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 16) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 8 ) & 0xff);
	sendbuff[i++] = (uint8_t)( (CarID >> 0 ) & 0xff);
	sendbuff[i++] = 13;
	sendbuff[i++] = VOLTAGE;
	sendbuff[i++] = (uint8_t)( 0xff & (time_std_s >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_std_s >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_std_s >> (2*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_std_s >> (3*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (voltages >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (voltages >> (1*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (current_battery >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (current_battery >> (1*8)));
	sendbuff[i++] = battery_remaining;
	sendbuff[i++] = (uint8_t)( 0xff & (time_can_run >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (time_can_run >> (1*8)));
	CheckSum = GetCheckSum(sendbuff,13);
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (0*8)));
	sendbuff[i++] = (uint8_t)( 0xff & (CheckSum >> (1*8)));
	return SendData(sendbuff, i);
}
/**
 *@brief  ���ݿ���ģʽѡ���ܵ������ݻ��Ǿ���Ѳ�ߵõ���������Ϊ��СС�����ٶȺͽ��ٶ�
 *@param  ����ģʽ
 *@retvel none
 */
uint8_t LastAng=0,Leftime=0,rightime=0;
//void CarSpeedSelection(uint8_t mod, int16_t *Vel,int16_t *Ang)
void CarSpeedSelection(void* addr[7])
{
	static uint8_t LineWidthLast = LINE_WIDTH_MIN;
	static int8_t	JumpVal = 0;
	int8_t Yaw = 0;
	uint8_t *pMode,mod,*pLinePost,*pLineWidth;
	int16_t *Vel,*Ang;
	uint8_t LineWidth = 0,LinePosition = 0;
	#ifdef HollowStopLine
	uint8_t LineCnt,*pLineGap,*pLineCnt;
	#endif
	pMode					= addr[0];	
	#ifdef HollowStopLine
	pLineCnt			=	addr[1];
	pLineGap			=	addr[2];
	#endif
	Vel						= addr[3];
	Ang						= addr[4];
	pLinePost			= addr[5];
	pLineWidth		=	addr[6];
	
	mod						=	*pMode;
	LineWidth			=	*pLineWidth;
	LinePosition	= *pLinePost;
	#ifdef HollowStopLine
	LineCnt				=	*pLineCnt;
	#endif
	#ifndef HollowStopLine
	if(MOD_LINE_FOLLOW == mod)//Ѳ��ģʽ
	{		
		if((LineWidth > LINE_WIDTH_MIN) && (LineWidth < LINE_WIDTH_MAX))
		{
			LineWidtherTime = 0;
			Yaw = 64 - LinePosition;
			if(Yaw < 0)
				Yaw = -Yaw;
			/*----------------------------�������ٶ�-----------------------*/
			//*Vel = AUTO_VEL_MAX*(LineWidth-LINE_WIDTH_MAX)/(LINE_WIDTH_MIN-LINE_WIDTH_MAX);
			*Vel = (LINE_WIDTH_MAX - LineWidth)*(64 - Yaw)/120;
			/*------------------------------�ٶ�С��һ���̶Ⱥ��ٶ���Ϊ�㶨ֵ------------------*/
			if(LineWidth > (LINE_WIDTH_MAX*CRITICAL_VAL/(CRITICAL_VAL+2)))
			{
				*Vel =  (LINE_WIDTH_MAX - (LINE_WIDTH_MAX*CRITICAL_VAL/(CRITICAL_VAL+2)))/10;
				LineGraduChge = 1;
			}
			
			/*----------------------------������ٶ�-----------------------*/
			if((LINE_POS_MAX > LinePosition) && (LINE_POS_MIN < LinePosition))
			{
				*Ang = (int16_t)((float)AUTO_ANG_MAX*(64 - LinePosition)/64);
			}
			else
			{
				*Ang = 0;
			}	
		
      /*---------------------------��ֹ�ٶȷ���ͻ��-----------------------*/			
			JumpVal = LineWidth - LineWidthLast;
			if(JumpVal < 0)
				JumpVal = -JumpVal;
			LineWidthLast = LineWidth;
			/*-----------------------��ʱû���ã�����ٶȿ�ʱ�п�������----------------------*/
//			memset(WriteBuffer,0,sizeof(WriteBuffer));
//			sprintf(WriteBuffer,"\nѲ�����ݣ�S=%d,A=%d",*Vel,*Ang);
//			fileWrite(WriteBuffer,"0:Ѳ���ٶ�����ٶ�.txt");
		}
		else
		{
			if(LineGraduChge)
			{
				ModeReg = 0;
				LineGraduChge = 0;
			}
			else
			{
				LineWidtherTime++;
				if( LineWidtherTime > LINEWIDTHCNT)
				{
					ModeReg = 3;
					LineWidtherTime = 0;
				}
			}
			*Vel = 0;
			*Ang = 0;
		}
	}	
	#else
	if(MOD_LINE_FOLLOW == mod)//Ѳ��ģʽ
	{
		switch(LineCnt)
		{
			case 0x01:
				LineWidtherTime = 0;
				if(LineWidth < LINE_WIDTH_MAX)
				{
					Yaw = 64 - LinePosition;
					if(Yaw < 0)
						Yaw = -Yaw;
					/*---------------------------�����ٶ�--------------------------*/
					*Vel = (LINE_WIDTH_MAX - LineWidth)*(64 - Yaw)/120;
					/*----------------------------������ٶ�-----------------------*/
					if((LINE_POS_MAX > LinePosition) && (LINE_POS_MIN < LinePosition))
					{
						*Ang = (int16_t)((float)AUTO_ANG_MAX*(64 - LinePosition)/64);
					}
					else
					{
						*Ang = 0;
					}	
				}
				else
				{
					*Vel = (64 - Yaw)/120;
					if((LINE_POS_MAX > LinePosition) && (LINE_POS_MIN < LinePosition))
					{
						*Ang = (int16_t)((float)AUTO_ANG_MAX*(64 - LinePosition)/64);
					}
					else
					{
						*Ang = 0;
					}	
				}
				  /*---------------------------��ֹ�ٶȷ���ͻ��-----------------------*/			
				JumpVal = LineWidth - LineWidthLast;
				if(JumpVal < 0)
					JumpVal = -JumpVal;
				LineWidthLast = LineWidth;
				/*-----------------------��ʱû���ã�����ٶȿ�ʱ�п�������----------------------*/
			break;
			case 0x02:
				LineWidtherTime = 0;
				if(*pLineGap < LINESTOPGAP)
				{
					*Vel = LINE_WIDTH_MAX*CRITICAL_VAL/(CRITICAL_VAL+2)/10;
					if((LINE_POS_MAX > LinePosition) && (LINE_POS_MIN < LinePosition))
					{
						*Ang = (int16_t)((float)AUTO_ANG_MAX*(64 - LinePosition)/64);
					}
					else
					{
						*Ang = 0;
					}
				}
				else
				{
					*Vel = 0;
					*Ang = 0;
					FindLineStatu = 0;
					ModeReg = 0;
				}	
			  /*---------------------------��ֹ�ٶȷ���ͻ��-----------------------*/			
			JumpVal = LineWidth - LineWidthLast;
			if(JumpVal < 0)
				JumpVal = -JumpVal;
			LineWidthLast = LineWidth;
			/*-----------------------��ʱû���ã�����ٶȿ�ʱ�п�������----------------------*/
			break;
			default:
				LineWidtherTime++;
				if( LineWidtherTime > LINEWIDTHCNT)
				{
					ModeReg = 3;
					LineWidtherTime = 0;
				}
				*Vel = 0;
				*Ang = 0;
			break;
		}
	}
	#endif
	else if(MOD_LINE_BACKFOLLOW ==  mod)
	{
		*Vel = -BackCarSpd;
		*Ang = 0;
	}
	else if(MOD_UP_CONTROL == mod) //�ϲ����
	{
		*Vel = Vel_Set;
		*Ang = Ang_Set;
//		memset(WriteBuffer,0,sizeof(WriteBuffer));
//		sprintf(WriteBuffer,"\n�ٶȽ��ٶȣ�S=%d,A=%d",Vel_Set,Ang_Set);
//		fileWrite(WriteBuffer,"0:�ϲ�����ٶ�����ٶ�.txt");
	}
	else
	{
		*Vel = 0;
		*Ang = 0;
	}

}
























