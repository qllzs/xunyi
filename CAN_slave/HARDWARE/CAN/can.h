#ifndef __CAN_H
#define __CAN_H
#include "can.h"


/******功能码*******/
//车轮电机通信功能码
#define		FuncVehWhel						0x00
//时间同步/广播
#define		FuncSyncMsg						0x01
//主机给从机发送命令
#define		FuncM2SlavCmd					0x02
//从机回复收到命令指令
#define		FuncSlav2MRsp					0x03
//主机查看从机参数
#define		FuncMInqSlavPara				0x04
//从机回复主机参数
#define		FuncSlav2MParaRsp				0x05
//主机设置从机的参数
#define		FuncMSetSlavPara				0x06
//从机回复主机参数设置
#define		FuncSlav2MSetParaRsp			0x07
//从机主动发给主机的数据
#define 	FuncSlav2MData					0x08
//主机查询从机的程序版本
#define 	FuncMInqSlaVers					0x09
//从机回复自身程序版本
#define   	FuncSlave2MVers					0x0A

/**********End**********/



/******从机地址**********/
//主控板地址
#define		AddrMaster						0x00
//电机的左侧地址
#define		AddrMectorLft					0x01
//电机的右侧地址
#define		AddrMectorRht					0x02
//巡线地址
#define		AddrCCDModul					0x03
//箱门电机模块地址
#define		AddrMotorModul					0x04
//扩展IO模块
#define		AddrIOModul						0x05

/*******End********/

typedef struct  _recvdata{
	uint8_t		FuncNum; //功能码
	uint8_t		EquipNum; //设备地址
	uint8_t		Param[8];  //参数列表	
}StrCanRecvData;

void CanModeInit(void);
void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat);
void Clear_RxMes(CanRxMsg *RxMessage);
void DoActionFunc(StrCanRecvData* p);
#endif
