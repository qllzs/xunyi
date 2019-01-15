#include "GlobalParam.h"

CanRxMsg RxMessage;	 //接收缓存
StrCanRecvData	CanRecvData;
extern uint8_t	DoActionFlag;

void CanModeInit(void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure; 
	NVIC_InitTypeDef  		NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//初始化IO

	GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//初始化IO
	
	
	//CAN单元设置
	CAN_InitStructure.CAN_TTCM							=	DISABLE;//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM							=	ENABLE;//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM							=	ENABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART							=	DISABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM							=	DISABLE;//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP							=	DISABLE;//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode							=	CAN_Mode_Normal;//模式设置：,普通模式; 
	
	//CAN 1M
	CAN_InitStructure.CAN_SJW							=	CAN_SJW_2tq;//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1							=	CAN_BS1_5tq; //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2							=	CAN_BS2_3tq;	//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler						=	8;  //分频系数(Fdiv)为brp
	CAN_Init(CAN1, &CAN_InitStructure);        	//初始化CAN1 
	
	
	CAN_FilterInitStructure.CAN_FilterNumber			=	0;	//过滤器0
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdList; 	//过滤器列表模式
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_16bit; 	// 
	//根据不同的配置程序完成不同的定义
	//巡线模式
	#if defined LinePatrolModule
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncSyncMsg<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrCCDModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrCCDModul) << 5);
	#elif defined	BoxMotorModule
	//箱门控制
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	((FuncSyncMsg<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrMotorModul) << 5);
	#elif defined IOExtendModule
	//IO扩展板
	CAN_FilterInitStructure.CAN_FilterIdHigh			= 	((FuncSyncMsg<<7 | AddrIOModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	((FuncM2SlavCmd<<7 | AddrIOModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	((FuncMInqSlavPara<<7 | AddrIOModul) << 5);
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	((FuncMSetSlavPara<<7 | AddrIOModul) << 5);
	#endif
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;//激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/**********************************************************************************
	*如果需要匹配的更多的ID列表，则可以继续增加过滤器，关联到CAN_Filter_FIFO1
	***********************************************************************************/
	CAN_FilterInitStructure.CAN_FilterNumber			=	1;	//过滤器1
	CAN_FilterInitStructure.CAN_FilterMode				=	CAN_FilterMode_IdList; 	//过滤器列表模式
	CAN_FilterInitStructure.CAN_FilterScale				=	CAN_FilterScale_16bit; 	// 
	//根据不同的配置程序完成不同的定义
	//巡线模式
	#if defined LinePatrolModule
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncMInqSlaVers<<7 | AddrCCDModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#elif defined	BoxMotorModule
	//箱门控制
	CAN_FilterInitStructure.CAN_FilterIdHigh			=	((FuncMInqSlaVers<<7 | AddrMotorModul) << 5);
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#elif defined IOExtendModule
	//IO扩展板
	CAN_FilterInitStructure.CAN_FilterIdHigh			= ((FuncMInqSlaVers<<7 | AddrIOModul) << 5);	
	CAN_FilterInitStructure.CAN_FilterIdLow				=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh		=	0x00;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow			=	0x00;
	#endif
	
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment	=	CAN_Filter_FIFO1;//过滤器1关联到FIFO1
	CAN_FilterInitStructure.CAN_FilterActivation		=	ENABLE;//激活过滤器1
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/**********************************************************************************************
	*过滤器1配置完成
	************************************************************************************************/
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_EPV, ENABLE); 
}

/****************************************************************************
功能：CAN1发送数据
参数：id-设备ID号；len-发送数据长度；*dat-待发送的数据数组
*/
void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat)
{
	uint8_t ubCounter = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE = CAN_ID_STD;	    //标准ID   
	TxMessage.RTR = CAN_RTR_DATA;   //设置为数据帧（或远程帧为CAN_RTR_Remote）
	TxMessage.DLC = len;	          
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
  {
    TxMessage.Data[ubCounter] = 0;
		TxMessage.Data[ubCounter] = dat[ubCounter];
  }
	
	CAN_Transmit(CAN1, &TxMessage);   // 开始传送数据
}


/****************************************************************************
功能：接收结构体清零 
参数：接收结构体指针CanRxMsg *RxMessage
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
		//同步或升级程序
		case 0x01:
			
		break;
		//M下发的命令
		case 0x02:
			if(p->EquipNum == AddrMotorModul)//箱门电机，则需要执行对应的动作
			{
				BoxMotorControl(p->Param[0]);
			}
			if(p->EquipNum == AddrCCDModul)  //CCD模块
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
		//查寻固化参数
		case 0x04:
			ret = ee_ReadBytes(p->Param,0,6);   //读取参数
			if(ret)
			{
				CAN1_SendMesg((FuncSlav2MParaRsp<<7|p->EquipNum),6,p->Param); //回复固化参数
			}
		
		break;
		//设置固化参数
		case 0x06:
			ret = ee_WriteBytes(p->Param,0,6);  //写固化参数
			if(ret)
			{
				CAN1_SendMesg((FuncSlav2MSetParaRsp<<7|p->EquipNum),6,p->Param); //回复固化参数
			}
		break;
		//查询程序版本
		case 0x09:
			ret = ee_ReadBytes(p->Param,6,6);   //读取参数
			if(ret)
			{
				CAN1_SendMesg((FuncSlave2MVers<<7|p->EquipNum),6,p->Param); //回复固化参数
			}
		break;
		default:
			
		break;
		
	}
}

//uint8_t FuncNum=0,ModuleNum=0; //功能码，信息板载编号
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	uint8_t tempi;
	// 检测 错误被动中断屏蔽 标志位并及时清零
  if(CAN_GetITStatus(CAN1, CAN_FLAG_EPV))
	{
		CAN_ClearITPendingBit(CAN1, CAN_FLAG_EPV);
	}
	
	if((CAN_MessagePending(CAN1, CAN_FIFO0) != 0)) //检查FIFO0里面是否有数据
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
	
	if((CAN_MessagePending(CAN1, CAN_FIFO1) != 0)) //检查FIFO1里面是否有数据
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

