#include "bsp_WheelMotor_CAN.h"
#ifdef METEC_MOTOR


uint32_t Motor_id_L = 1,Motor_id_R = 1;
int32_t Speed_L_RPM = 0,Speed_R_RPM = 0;//�����ʵʱת�٣���λ��0.1RPM
//CAN����Ƶ��ö�ٶ���
enum CAN_Frequency_enum
{
	CAN_Frequency_50KHz = 80,
	CAN_Frequency_125KHz = 32,
	CAN_Frequency_250KHz = 16,
	CAN_Frequency_500KHz = 8,
	CAN_Frequency_1000KHz = 4,
} CAN_Frequency;
CanRxMsg RxMessage;				   //���ջ�����
/****************************************************************************
���ܣ�CAN �� NVIC ����
��������
*/
static void CAN1_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  // USB_LP_CAN1_RX0_IRQHandler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	// FIFO0��Ϣ�Һ��ж�����;
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_EPV, ENABLE);   	
	// CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);   	
}

/****************************************************************************
���ܣ�CAN1�������ü�CANģʽ��ʼ��
��������
*/
static void CAN1_Config(enum CAN_Frequency_enum CAN_Frq)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	
	/* GPIO ʱ�Ӽ�����Դ */
	RCC_APB2PeriphClockCmd(CAN_GPIO_AF_CLK | CAN_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //�����������
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;  // GPIO_Pin_12; //PA12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //��������
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;         //GPIO_Pin_11; //PA11
	GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);

	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;    //ʧ��ʱ�䴥��ģʽ
	CAN_InitStructure.CAN_ABOM = ENABLE;    //ʧ���Զ����߹���
	CAN_InitStructure.CAN_AWUM = ENABLE;    //ʧ��˯��ģʽͨ���������
	CAN_InitStructure.CAN_NART = DISABLE;    //ʧ�ܷ��Զ��ش���ģʽ��Ҳ���ǻ��Զ��ش��䣩
	CAN_InitStructure.CAN_RFLM = DISABLE;    //ʧ�ܽ���FIFO����ģʽ�������ݻḲ�Ǿ�����
	CAN_InitStructure.CAN_TXFP = DISABLE;    //���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;       //����ͨģʽ����չģʽ
	//CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq; //����ͬ����Ծ��� 1 ��ʱ�䵥λ
  /* ����������, ��APB1��ʱ��Ƶ����36MHZ��ʱ�� �����ʵĹ�ʽΪ�� */
  /* ������(Kpbs) = 36M / ((CAN_BS1 + CAN_BS2 + 1) *  CAN_Prescaler) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //ʱ��� 1 Ϊ8 ��ʱ�䵥λ 
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; //ʱ��� 2 Ϊ7 ��ʱ�䵥λ
	CAN_InitStructure.CAN_Prescaler = CAN_Frq;	 

	CAN_Init(CANx, &CAN_InitStructure);

	#ifdef CAN_RX0_INT_ENABLE
		CAN1_NVIC_Config();
	#endif    
}


/****************************************************************************
���ܣ�CAN1ɸѡ������ 
��������
*/
static void CAN1_Config16BitFilter(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	uint16_t mask = 0xFFFc;

	CAN_FilterInitStructure.CAN_FilterNumber = 1;	                  //������1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //IDģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// �Ĵ���������Ϊ16λ 

	CAN_FilterInitStructure.CAN_FilterIdHigh= ((u16)0x1<<5);		  //Ҫɸѡ��ID��λ 
	//CAN_FilterInitStructure.CAN_FilterIdLow= (0X00<<5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= (mask<<5);			//ƥ��
	//CAN_FilterInitStructure.CAN_FilterMaskIdLow= (mask<<5);	

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;  //ʹ�ܹ�����1
	CAN_FilterInit(&CAN_FilterInitStructure);
}
//static void CAN1_Config32BitFilter(void)
//{
//	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

//	CAN_FilterInitStructure.CAN_FilterNumber = 1;	                  //������1
//	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //IDģʽ
//	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// �Ĵ���������Ϊ16λ 

//	CAN_FilterInitStructure.CAN_FilterIdHigh= 0;		  //Ҫɸѡ��ID��λ 
//	CAN_FilterInitStructure.CAN_FilterIdLow= 0;
//	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0;			//ƥ��
//	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0;	

//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  
//	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;  //ʹ�ܹ�����1
//	CAN_FilterInit(&CAN_FilterInitStructure);
//}


/****************************************************************************
���ܣ�CAN1��������
������id-�豸ID�ţ�len-�������ݳ��ȣ�*dat-�����͵���������
*/
void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat)
{
	uint8_t ubCounter = 0;
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	//TxMessage.ExtId = id;
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

static uint16_t getCRC16(uint8_t *ptr,uint8_t len)
{
	uint8_t i;
	uint16_t crc = 0xFFFF;
	if(len==0)
	{
		len = 1;
	}
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
		ptr++;
	}
	return(crc);
}
/**
 *@brief��ͨ�����߷�����Ϣ�ĺ���
*@param��Ad: ���ַ   CmdID:����ID     param1:����1    param2:����2
 *@retval��none
 */
void SendMessage2MotorDriver(uint8_t Ad,uint8_t CmdID,int16_t param1,int16_t param2)
{
	uint8_t data[8] = {0},i = 0;
	uint16_t CRC_Value = 0;
	data[i++] = Ad;
	data[i++] = CmdID;
	data[i++] = (param2 >> 8)&0xff;
	data[i++] = (param2 >> 0)&0xff;
	data[i++] = (param1 >> 8)&0xff;
	data[i++] = (param1 >> 0)&0xff;
	CRC_Value = getCRC16(data,6);
	data[i++] = (CRC_Value >> 0)&0xff;
	data[i++] = (CRC_Value >> 8)&0xff;
	CAN1_SendMesg(Ad, 8, data);
}
/**
 *@brief��С�����������ʼ������
 *@param��none
 *@retval��none
 */
void Metec_MOTOR_Init(void)
{
	CAN1_Config(CAN_Frequency_500KHz);
	CAN1_Config16BitFilter();
	//CAN1_Config32BitFilter();
	delay_ms(1000);
	SendMessage2MotorDriver(MOTOR_L,PRIM_ENABLE,0,0);
	SendMessage2MotorDriver(MOTOR_R,PRIM_ENABLE,0,0);
	
	SendMessage2MotorDriver(MOTOR_L,PRIM_SETVELOCITY,0,0);	
	SendMessage2MotorDriver(MOTOR_R,PRIM_SETVELOCITY,0,0);
}
void USB_LP_CAN1_RX0_IRQHandler(void)
{
//	int i2 = 0;
	
	// ��� ���󱻶��ж����� ��־λ����ʱ����
  if(CAN_GetITStatus(CANx, CAN_FLAG_EPV))
	{
		CAN_ClearITPendingBit(CANx, CAN_FLAG_EPV);
	}
	
	if((CAN_MessagePending(CANx, CAN_FIFO0) != 0)) //���FIFO0�����Ƿ�������
	{

		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
		if(RxMessage.Data[0] == MOTOR_L)//������յ�����
		{
			if(RxMessage.Data[1] == PRIM_GETACTVELOCITY)//�ٶ�
			{
				Speed_L_RPM = (RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0);
			}
			if(RxMessage.Data[1] == PRIM_GETERROR)
			{
				if(((RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0)) & FAULT_UNDER_VOLTAGE)
					FaultReg |= (3<<0);
				if(((RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0)) & FAULT_I2T_ERROR)
					FaultReg |= (5<<0);
			}
		}
		else   //�Ҳ����յ�����
		{
			if(RxMessage.Data[1] == PRIM_GETACTVELOCITY)//�ٶ�
			{
				Speed_R_RPM = 0 - ((RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0));
			}
			if(RxMessage.Data[1] == PRIM_GETERROR)
			{
				if(((RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0)) & FAULT_UNDER_VOLTAGE)
					FaultReg |= (3<<4);
				if(((RxMessage.Data[2] << 24) | (RxMessage.Data[3] << 16) | (RxMessage.Data[4] << 8) | (RxMessage.Data[5] << 0)) & FAULT_I2T_ERROR)
					FaultReg |= (5<<0);
			}
		}
	}
	
}


#endif  //METEC_MOTOR
