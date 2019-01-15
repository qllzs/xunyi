#ifndef             __PROTOCOL_H__
#define             __PROTOCOL_H__
#include "bsp_include.h"
/******************宏定义声明**********************/
//小车ID的设定
#define CAR_ID           0x55660100 //默认的小车ID
#define BROADCAST_ID     0xFFFFFFFF
#define	BackCarSpd			 8
//下行消息ID的宏定义
#define PULSE_DOWN         0x00
#define SPEED_CONFIG       0x01
#define INSTRUCTION        0x02
#define PARAMETER_CONFIG   0x03
//上行消息ID的宏定义
#define PULSE_UP               0x90
#define SPEED_FEEDBACK         0x91
#define ALARM                  0x92
#define INSTRUCTION_FEEDBACK   0x93
#define VOLTAGE                0x94
//control command table
#define 	CMD_STOP_EMC         0X00
#define 	CMD_MODE_CTL         0X01
#define 	CMD_BOX_STATUS       0X02
#define 	CMD_LINE_STATUS      0X03

#define 	CMD_BOX_STATUS_CTL      0
#define 	CMD_BOX_STATUS_QURY     1
#define 	CMD_BOX_STATUS_CTL_ON   1
#define 	CMD_BOX_STATUS_CTL_OFF  0

#define 	CMD_MODE_CTL_CONTROLER    		0
#define 	CMD_MODE_CTL_LINE_FOLLOW  		1
#define 	CMD_MODE_CTL_LINE_BACKFOLLOW  2

#define 	CMD_STOP_EMC_ON          1
#define 	CMD_STOP_EMC_OFF         0
/* config command table   */
#define CFG_ANTCAR_ID            0
#define CFG_ALARM                1
#define CFG_MAX_SPEED            2
/********告警信息表*********/
#define COLLISION_ID             2
#define MOTOR_ID                 1
#define BATTERY_ID               3
#define	BOX_ID									 4
/********告警级别 1：通知,2：警告,3：紧急*********/
#define ALARM_LEVEL_NOTICE       1
#define ALARM_LEVEL_WARNING      2
#define ALARM_LEVEL_EMC          3

/**********mode控制宏定义0=上层控制； 1=巡线模式； 2=倒车巡线  ；3=急停 *********/
#define MOD_UP_CONTROL           0
#define MOD_LINE_FOLLOW          1
#define MOD_LINE_BACKFOLLOW 		 2
#define MOD_EMC_STOP             3

/********************指令反馈信息状态表0=失败；1=进行中；2完成*********************/
#define INSTRUCTION_FEEDBACK_FAIL    0
#define INSTRUCTION_FEEDBACK_ONGOING 1
#define INSTRUCTION_FEEDBACK_SUCCEED 2

/*--------------------自动寻线相关的宏定义----------------------*/
#define AUTO_ANG_MAX                  120
#define AUTO_VEL_MAX                  20
#define LINE_WIDTH_MIN								4
#define LINE_WIDTH_MAX								32
#define	CRITICAL_VAL									3
#define	LINE_POS_MIN									6
#define	LINE_POS_MAX									122
#define	LINEWIDTHCNT									10
#define	LINESTOPGAP										20  
/******************结构体定义**********************/
//下行结构体定义
struct Instruction_Struct
{
	uint32_t cmd;
	uint8_t para1;
	uint8_t para2;
	uint8_t para3;
	uint8_t para4;
};
struct ParaConfig_Struct
{
	uint32_t para_id;
	uint8_t value1;
	uint8_t value2;
	uint8_t value3;
	uint8_t value4;
};
//上行结构体定义
struct Pulse_Up_Struct
{
	uint32_t ID;
	uint8_t len;
	uint8_t msg_id;
	uint32_t time_stamp;
	uint8_t mode;
	uint16_t check_sum;
};
struct SpeedFeedBack_Struct
{
	uint32_t ID;
	uint8_t len;
	uint8_t msg_id;
	int16_t velocity;
	int16_t angular;
	uint16_t check_sum;
};
struct Alarm_struct
{
	uint32_t ID;
	uint8_t len;
	uint8_t msg_id;
	uint32_t time_stamp;
	uint32_t alarm_ID;
	uint8_t alarm_level;
	uint32_t paral;
	uint16_t check_sum;
};
struct InstructionFeedBack_Struct
{
	uint32_t ID;
	uint8_t len;
	uint8_t msg_id;
	uint8_t status;
	uint8_t degree;
	uint16_t check_sum;
};
struct PowerConsume_Struct
{
	uint32_t ID;
	uint8_t len;
	uint8_t msg_id;
	uint32_t time_std_s;
	uint16_t voltages;
	uint16_t current_battery;
	uint8_t battery_remaining;
	uint16_t time_can_run;
	uint16_t check_sum;
};

/******************变量声明**********************/
extern volatile uint32_t CarID;
extern volatile uint32_t TimeStampReg;
extern volatile uint8_t  ModeReg;
extern volatile uint8_t  _Line_Sta;
//extern uint8_t LineWidth,LinePosition;

extern volatile uint32_t Time_Stamp_Set;
extern volatile int16_t Vel_Set;
extern volatile int16_t Ang_Set;
extern volatile struct Instruction_Struct Instruction_Structure;
extern volatile struct ParaConfig_Struct ParaConfigStructure;
extern uint8_t InstructionDoOverFlag;
//上行数据结构体数据定义
extern struct Pulse_Up_Struct Pulse_Up_Structure;
extern struct SpeedFeedBack_Struct SpeedFeedBack_Structure;
extern struct Alarm_struct Alarm_structure;
extern struct InstructionFeedBack_Struct InstructionFeedBack_Structure;
extern struct PowerConsume_Struct PowerConsume_Structure;
/******************函数声明**********************/

int8_t ReceiveDataAnalysis(uint8_t DataBuff[30]);
int8_t SendPulseMessageToHost(uint32_t time_stamp,uint8_t CarMode, uint8_t BoxStatus,uint8_t Car_status);
int8_t SendSpeedMessageToHost(uint16_t velocity , uint16_t angular);
int8_t SendAlarmMessageToHost(uint32_t time_stamp,uint32_t alarm_ID, uint8_t alarm_level,uint32_t alarm_paral);
int8_t SendInstructionFeedbackMessageToHost(uint8_t status,uint8_t degree);
int8_t  SendVoltageMessageToHost(uint32_t time_std_s,uint16_t voltages,uint16_t current_battery,uint8_t battery_remaining,uint16_t time_can_run);
//void CarSpeedSelection(uint8_t mod, int16_t *Vel,int16_t *Ang);
void CarSpeedSelection(void* addr[7]);
uint8_t Do_Instruction(volatile struct Instruction_Struct Instruction);

/****************************************/


















#endif    

