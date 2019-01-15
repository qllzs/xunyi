#include "protocol.h"
/***********************全局变量定义*********************************/
uint8_t InstructionDoOverFlag = 1;//指令执行完毕标志，1表示执行完毕 0表示未执行完毕
uint8_t LineWidtherTime=0;
volatile uint32_t CarID = CAR_ID;
volatile uint32_t TimeStampReg = 0;
volatile uint8_t  ModeReg = 0;//运行模式控制寄存器，0=上层控制； 1=巡线模式；2=  巡线后退;3=急停
volatile uint8_t  _Line_Sta = 0;//寻线检测状态1=检测到线；0=没有检测到线
volatile uint8_t  LineGraduChge = 0; //线逐渐变化 0没有经历 1:经历 ;在模式切换后清零
volatile uint8_t UpDownChangeMode =0 ; //0:从巡线切到上层控制。1：从上层控制切到巡线模式，如果没有从巡线切到上层控制则一直为1
//下行数据定义
volatile uint32_t Time_Stamp_Set = 0;
volatile int16_t Vel_Set = 0;
volatile int16_t Ang_Set = 0;
volatile struct Instruction_Struct Instruction_Structure;
volatile struct ParaConfig_Struct ParaConfigStructure;
//上行数据结构体数据定义
struct Pulse_Up_Struct Pulse_Up_Structure;
struct SpeedFeedBack_Struct SpeedFeedBack_Structure;
struct Alarm_struct Alarm_structure;
struct InstructionFeedBack_Struct InstructionFeedBack_Structure;
struct PowerConsume_Struct PowerConsume_Structure;
/*
 *功能：得到校验和
 *参数：data-传入的数据指针，len-数据长度
 *返回：check_sum 返回校验和
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
/*功能：对收到的数据进行解码
*参数: Buff_Rx_Final:收到数据的指针 MessageRevFlag:数据接收标志位
*返回：1:解析成功；-1:解析失败；
 */
int8_t ReceiveDataAnalysis(uint8_t DataBuff[30])
{
	uint8_t msg_id = DataBuff[5];	
	switch (msg_id)//判断消息ID
	{
		case PULSE_DOWN:
		{
			Time_Stamp_Set = (uint32_t)( DataBuff[6] | (DataBuff[7] << 8) | (DataBuff[8] << 16) | (DataBuff[9] << 24) );
			WatchDog[PULSE_WATCHDOG] = PULSE_WDT_TIMEOUT;
			FaultReg &=  ~PULSE_BIT;//将故障寄存器相应位清零
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
			return -1;//校验失败
	}
	return 1;
}
/*
*@brief:指令处理函数
*@param:Instruction --指令结构体
*@ret:返回1-执行完毕
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
//					sprintf(WriteBuffer,"\n上层控制指令：M=%d",Instruction.para1);
//					fileWrite(WriteBuffer,"0:收到上层控制指令.txt");
					break;
				}
				case CMD_BOX_STATUS:
				{
					
					if(Instruction.para1 == CMD_BOX_STATUS_CTL)
					{
						BoxControl(Instruction.para2);
					}
					else//查询箱门状态指令
					{

					}
					break;
				}
				case CMD_LINE_STATUS: //不用管
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
	//将小车ID进行赋值到buffer
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
 *@brief  根据控制模式选择将受到的数据还是经过巡线得到的数据作为最小小车的速度和角速度
 *@param  控制模式
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
	if(MOD_LINE_FOLLOW == mod)//巡线模式
	{		
		if((LineWidth > LINE_WIDTH_MIN) && (LineWidth < LINE_WIDTH_MAX))
		{
			LineWidtherTime = 0;
			Yaw = 64 - LinePosition;
			if(Yaw < 0)
				Yaw = -Yaw;
			/*----------------------------计算线速度-----------------------*/
			//*Vel = AUTO_VEL_MAX*(LineWidth-LINE_WIDTH_MAX)/(LINE_WIDTH_MIN-LINE_WIDTH_MAX);
			*Vel = (LINE_WIDTH_MAX - LineWidth)*(64 - Yaw)/120;
			/*------------------------------速度小到一定程度后将速度设为恒定值------------------*/
			if(LineWidth > (LINE_WIDTH_MAX*CRITICAL_VAL/(CRITICAL_VAL+2)))
			{
				*Vel =  (LINE_WIDTH_MAX - (LINE_WIDTH_MAX*CRITICAL_VAL/(CRITICAL_VAL+2)))/10;
				LineGraduChge = 1;
			}
			
			/*----------------------------计算角速度-----------------------*/
			if((LINE_POS_MAX > LinePosition) && (LINE_POS_MIN < LinePosition))
			{
				*Ang = (int16_t)((float)AUTO_ANG_MAX*(64 - LinePosition)/64);
			}
			else
			{
				*Ang = 0;
			}	
		
      /*---------------------------防止速度发生突变-----------------------*/			
			JumpVal = LineWidth - LineWidthLast;
			if(JumpVal < 0)
				JumpVal = -JumpVal;
			LineWidthLast = LineWidth;
			/*-----------------------暂时没有用，如果速度快时有卡顿再用----------------------*/
//			memset(WriteBuffer,0,sizeof(WriteBuffer));
//			sprintf(WriteBuffer,"\n巡线数据：S=%d,A=%d",*Vel,*Ang);
//			fileWrite(WriteBuffer,"0:巡线速度与角速度.txt");
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
	if(MOD_LINE_FOLLOW == mod)//巡线模式
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
					/*---------------------------计算速度--------------------------*/
					*Vel = (LINE_WIDTH_MAX - LineWidth)*(64 - Yaw)/120;
					/*----------------------------计算角速度-----------------------*/
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
				  /*---------------------------防止速度发生突变-----------------------*/			
				JumpVal = LineWidth - LineWidthLast;
				if(JumpVal < 0)
					JumpVal = -JumpVal;
				LineWidthLast = LineWidth;
				/*-----------------------暂时没有用，如果速度快时有卡顿再用----------------------*/
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
			  /*---------------------------防止速度发生突变-----------------------*/			
			JumpVal = LineWidth - LineWidthLast;
			if(JumpVal < 0)
				JumpVal = -JumpVal;
			LineWidthLast = LineWidth;
			/*-----------------------暂时没有用，如果速度快时有卡顿再用----------------------*/
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
	else if(MOD_UP_CONTROL == mod) //上层控制
	{
		*Vel = Vel_Set;
		*Ang = Ang_Set;
//		memset(WriteBuffer,0,sizeof(WriteBuffer));
//		sprintf(WriteBuffer,"\n速度角速度：S=%d,A=%d",Vel_Set,Ang_Set);
//		fileWrite(WriteBuffer,"0:上层控制速度与角速度.txt");
	}
	else
	{
		*Vel = 0;
		*Ang = 0;
	}

}
























