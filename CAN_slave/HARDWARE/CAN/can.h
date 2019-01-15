#ifndef __CAN_H
#define __CAN_H
#include "can.h"


/******������*******/
//���ֵ��ͨ�Ź�����
#define		FuncVehWhel						0x00
//ʱ��ͬ��/�㲥
#define		FuncSyncMsg						0x01
//�������ӻ���������
#define		FuncM2SlavCmd					0x02
//�ӻ��ظ��յ�����ָ��
#define		FuncSlav2MRsp					0x03
//�����鿴�ӻ�����
#define		FuncMInqSlavPara				0x04
//�ӻ��ظ���������
#define		FuncSlav2MParaRsp				0x05
//�������ôӻ��Ĳ���
#define		FuncMSetSlavPara				0x06
//�ӻ��ظ�������������
#define		FuncSlav2MSetParaRsp			0x07
//�ӻ�������������������
#define 	FuncSlav2MData					0x08
//������ѯ�ӻ��ĳ���汾
#define 	FuncMInqSlaVers					0x09
//�ӻ��ظ��������汾
#define   	FuncSlave2MVers					0x0A

/**********End**********/



/******�ӻ���ַ**********/
//���ذ��ַ
#define		AddrMaster						0x00
//���������ַ
#define		AddrMectorLft					0x01
//������Ҳ��ַ
#define		AddrMectorRht					0x02
//Ѳ�ߵ�ַ
#define		AddrCCDModul					0x03
//���ŵ��ģ���ַ
#define		AddrMotorModul					0x04
//��չIOģ��
#define		AddrIOModul						0x05

/*******End********/

typedef struct  _recvdata{
	uint8_t		FuncNum; //������
	uint8_t		EquipNum; //�豸��ַ
	uint8_t		Param[8];  //�����б�	
}StrCanRecvData;

void CanModeInit(void);
void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat);
void Clear_RxMes(CanRxMsg *RxMessage);
void DoActionFunc(StrCanRecvData* p);
#endif
