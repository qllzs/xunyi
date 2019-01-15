#include "GlobalParam.h"

CanRxMsg RxMessage;	 //���ջ���
StrCanRecvData	CanRecvData;
extern uint8_t	DoActionFlag;

void CanModeInit(void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure; 
	NVIC_InitTypeDef  		NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//��ʼ��IO
	
	
	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM							=	DISABLE;//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM							=	ENABLE;//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM							=	ENABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART							=	DISABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM							=	DISABLE;//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP							=	DISABLE;//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode							=	CAN_Mode_Normal;//ģʽ���ã�,��ͨģʽ; 
	
	//CAN 1M
	CAN_InitStructure.CAN_SJW							=	CAN_SJW_2tq;//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1							=	CAN_BS1_5tq; //Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2							=	CAN_BS2_3tq;	//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler						=	8;  //��Ƶϵ��(Fdiv)Ϊbrp
	CAN_Init(CAN1, &CAN_InitStructure);        	//��ʼ��CAN1 
	
	
	CAN_FilterInitStructure.CAN_FilterNumber			=	0;	//������0
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdList; 	//�������б�ģʽ
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_16bit; 	// 
	//���ݲ�ͬ�����ó�����ɲ�ͬ�Ķ���
	//Ѳ��ģʽ
	#if defined LinePatrolModule
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncSyncMsg<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrCCDModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrCCDModul) << 5);
	#elif defined	BoxMotorModule
	//���ſ���
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	((FuncSyncMsg<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrMotorModul) << 5);
	#elif defined IOExtendModule
	//IO��չ��
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 	((FuncSyncMsg<<7 | AddrIOModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrIOModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrIOModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrIOModul) << 5);
	#endif
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;//���������0
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/**********************************************************************************
	*�����Ҫƥ��ĸ����ID�б�����Լ������ӹ�������������CAN_Filter_FIFO1
	***********************************************************************************/
	CAN_FilterInitStructure.CAN_FilterNumber			=	1;	//������1
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdList; 	//�������б�ģʽ
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_16bit; 	// 
	//���ݲ�ͬ�����ó�����ɲ�ͬ�Ķ���
	//Ѳ��ģʽ
	#if defined LinePatrolModule
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncMInqSlaVers<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#elif defined	BoxMotorModule
	//���ſ���
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	((FuncMInqSlaVers<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#elif defined IOExtendModule
	//IO��չ��
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncMInqSlaVers<<7 | AddrIOModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#endif
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO1;//������1������FIFO1
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;//���������1
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/**********************************************************************************************
	*������1�������
	************************************************************************************************/
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_EPV, ENABLE); 
}

/****************************************************************************
���ܣ�CAN1��������
������id-�豸ID�ţ�len-�������ݳ��ȣ�*dat-�����͵���������
*/
void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat)
{
	uint8_t ubCounter = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE = CAN_ID_STD;	    //��׼ID   
	TxMessage.RTR = CAN_RTR_DATA;   //����Ϊ����֡����Զ��֡ΪCAN_RTR_Remote��
	TxMessage.DLC = len;	          
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    TxMessage.Data[ubCounter] = 0;
		TxMessage.Data[ubCounter] = dat[ubCounter];
  }
	
	CAN_Transmit(CAN1, &TxMessage);   // ��ʼ��������
}


/****************************************************************************
���ܣ����սṹ������ 
���������սṹ��ָ��CanRxMsg *RxMessage
*/
void Clear_RxMes(CanRxMsg *RxMessage)
{
  static uint8_t ubCounter = 0;
  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    RxMessage->Data[ubCounter] = 0x00;
  }
}

void DoActionFunc(StrCanRecvData* p)
{
	uint8_t ret = 0;
	switch(p->FuncNum)
	{
		//ͬ������������
		case 0x01:
			
		break;
		//M�·�������
		case 0x02:
			if(p->EquipNum == AddrMotorModul)//���ŵ��������Ҫִ�ж�Ӧ�Ķ���
			{
				BoxMotorControl(p->Param[0]);
			}
			if(p->EquipNum == AddrCCDModul)  //CCDģ��
			{
				switch(p->Param[2])
				{
					case 0:
						LED_RED;
					break;
					case 1:
						LED_BLUE;
					break;
					case 2:
						LED_YELLOW;
					break;
					case 3:
						LED_PURPLE;
					break;
					default:
						LED_CYAN;
					break;
				}
			}
		break;
		//��Ѱ�̻�����
		case 0x04:
			ret = ee_ReadBytes(p->Param,0,6);   //��ȡ����
			if(ret)
			{
				CAN1_SendMesg((FuncSlav2MParaRsp<<7|p->EquipNum),6,p->Param); //�ظ��̻�����
			}
		
		break;
		//���ù̻�����
		case 0x06:
			ret = ee_WriteBytes(p->Param,0,6);  //д�̻�����
			if(ret)
			{
				CAN1_SendMesg((FuncSlav2MSetParaRsp<<7|p->EquipNum),6,p->Param); //�ظ��̻�����
			}
		break;
		//��ѯ����汾
		case 0x09:
			ret = ee_ReadBytes(p->Param,6,6);   //��ȡ����
			if(ret)
			{
				CAN1_SendMesg((FuncSlave2MVers<<7|p->EquipNum),6,p->Param); //�ظ��̻�����
			}
		break;
		default:
			
		break;
		
	}
}

//uint8_t FuncNum=0,ModuleNum=0; //�����룬��Ϣ���ر��
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint8_t tempi;
	// ��� ���󱻶��ж����� ��־λ����ʱ����
  if(CAN_GetITStatus(CAN1, CAN_FLAG_EPV))
	{
		CAN_ClearITPendingBit(CAN1, CAN_FLAG_EPV);
	}
	
	if((CAN_MessagePending(CAN1, CAN_FIFO0) != 0)) //���FIFO0�����Ƿ�������
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
		CanRecvData.FuncNum			= (RxMessage.StdId >> 7) & 0xFF;
		CanRecvData.EquipNum		=	RxMessage.StdId & 0x7F;
	
		for(tempi = 0;tempi < 8;tempi++)
		{
			CanRecvData.Param[tempi] = RxMessage.Data[tempi];
		}
		DoActionFlag = 1;
	}
	
	if((CAN_MessagePending(CAN1, CAN_FIFO1) != 0)) //���FIFO1�����Ƿ�������
	{
		CAN_Receive(CAN1, CAN_FIFO1, &RxMessage);
		CanRecvData.FuncNum			= (RxMessage.StdId >> 7) & 0xFF;
		CanRecvData.EquipNum		=	RxMessage.StdId & 0x7F;
	
		for(tempi = 0;tempi < 8;tempi++)
		{
			CanRecvData.Param[tempi] = RxMessage.Data[tempi];
		}
		DoActionFlag = 1;

	}
}

