
#include "bsp_OldMotor_CAN.h"
#ifdef OLD_WHEEL_MOTOR
uint8_t MotorFault=0;  // ������ϱ�־

uint32_t flag1 = 0;		 //���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ
uint32_t flag2 = 0;	
uint32_t flag3 = 0;	
uint32_t flag4 = 0;

uint8_t ReceiveBuff1[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff2[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff3[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff4[8] = {0,0,0,0,0,0,0,0};

uint8_t speedmode[8]={0x00, 0xFA, 0x00, 0x19, 0x00, 0x00, 0x00, 0x2F}; //�����ٶ�ģʽ
uint8_t upspeedtime[8]={0x00, 0xFA, 0x00, 0x13, 0x00, 0x00, 0x0A, 0x0B}; //���üӼ���ʱ��
uint8_t goalspeedzero[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
uint8_t goalspeed[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
uint8_t enablemotor[8]={0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F}; //ʹ�ܵ��
uint8_t stopmotor[8]={0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F};  //stop
uint8_t ResetMotorFault[8] = {0x00, 0xFA, 0x00, 0x15, 0x00, 0x00, 0x00, 0x7F};  // ��λ����

//�ı��ٶ�

uint8_t Config_Ki[8]={0x00, 0xFA, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00}; 
uint8_t Config_Kp[8]={0x00, 0xFA, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00}; 
uint8_t Config_Kd[8]={0x00, 0xFA, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00}; 



//static CanTxMsg TxMessage;			     //���ͻ�����
static CanRxMsg RxMessage;				   //���ջ�����

/*
������������
02 ������ 00 FF 00 EB 02 02 02 02
04 ��ѹ�� 00 FF 00 EB 04 04 04 04
08 ���������ϣ� 00 FF 00 EB 08 08 08 08
10 ���ȣ� 00 FF 00 EB 10 10 10 10
20 Ƿѹ�� 00 FF 00 EB 20 20 20 20
40 ����״̬�� 00 FF 00 EB 40 40 40 40
01 ����״̬�� 00 FF 00 EB 01 01 01 01
00 �ڲ��������룺���ڲ�����ģʽ�±���������� 00
80 �ⲿ���ƴ��룺���ⲿ����ģʽ�±���������� 80
*/

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
static void CAN1_Config(void)
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
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq; //����ͬ����Ծ��� 1 ��ʱ�䵥λ
  /* ����������, ��APB1��ʱ��Ƶ����36MHZ��ʱ�� �����ʵĹ�ʽΪ�� */
  /* ������(Kpbs) = 36M / ((CAN_BS1 + CAN_BS2 + 1) *  CAN_Prescaler) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //ʱ��� 1 Ϊ8 ��ʱ�䵥λ 
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; //ʱ��� 2 Ϊ7 ��ʱ�䵥λ
	CAN_InitStructure.CAN_Prescaler = 8;	 

	CAN_Init(CANx, &CAN_InitStructure);

	#ifdef CAN_RX0_INT_ENABLE
		CAN1_NVIC_Config();
	#endif    
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
	TxMessage.IDE = CAN_ID_STD;	    //��չID   
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
���ܣ�CAN1ɸѡ������ 
��������
*/
static void CAN1_Config16BitFilter(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	uint16_t mask = 0xFFF8;

	CAN_FilterInitStructure.CAN_FilterNumber = 1;	                  //������1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //IDģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// �Ĵ���������Ϊ16λ 

	CAN_FilterInitStructure.CAN_FilterIdHigh= ((u16)0x03<<5);		  //Ҫɸѡ��ID��λ 
	//CAN_FilterInitStructure.CAN_FilterIdLow= (0X00<<5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= (mask<<5);			//ƥ��
	//CAN_FilterInitStructure.CAN_FilterMaskIdLow= (mask<<5);	

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;  //ʹ�ܹ�����1
	CAN_FilterInit(&CAN_FilterInitStructure);
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
/****************************************************************************
���ܣ���챵����ʼ������-�ٶ�ģʽ����ʼ�ٶȣ����ʹ��
��������
*/
void Wheel_MOTOR_Init(void)
{
		Config_Kp[6] = KP/256;
		Config_Kp[7] = KP%256;
		Config_Ki[6] = KI/256;
		Config_Ki[7] = KI%256;
		Config_Kd[6] = KD/256;
		Config_Kd[7] = KD%256;
	
	  CAN1_SendMesg(ID1, 8, speedmode);
	  delay_ms(1);		
    CAN1_SendMesg(ID1, 8, upspeedtime);
	  delay_ms(1);	
    CAN1_SendMesg(ID1, 8, goalspeedzero);
	  delay_ms(1);	
    CAN1_SendMesg(ID1, 8, enablemotor);
	  delay_ms(1);	
		
    CAN1_SendMesg(ID2, 8, speedmode);
	  delay_ms(1);	
    CAN1_SendMesg(ID2, 8, upspeedtime);
	  delay_ms(1);	
    CAN1_SendMesg(ID2, 8, goalspeedzero);
	  delay_ms(1);	
    CAN1_SendMesg(ID2, 8, enablemotor);
	  delay_ms(1);	
		
    CAN1_SendMesg(ID3, 8, speedmode);
	  delay_ms(1);	
    CAN1_SendMesg(ID3, 8, upspeedtime);
	  delay_ms(1);	
    CAN1_SendMesg(ID3, 8, goalspeedzero);
	  delay_ms(1);	
    CAN1_SendMesg(ID3, 8, enablemotor);
	  delay_ms(1);	
		
    CAN1_SendMesg(ID4, 8, speedmode);
	  delay_ms(1);	
    CAN1_SendMesg(ID4, 8, upspeedtime);
	  delay_ms(1);	
    CAN1_SendMesg(ID4, 8, goalspeedzero);
	  delay_ms(1);	
    CAN1_SendMesg(ID4, 8, enablemotor);
	  delay_ms(10);
		//���õ����PID����
		CAN1_SendMesg(ID1, 8, Config_Kp);
	  delay_ms(1);
		CAN1_SendMesg(ID2, 8, Config_Kp);
	  delay_ms(1);
		CAN1_SendMesg(ID3, 8, Config_Kp);
	  delay_ms(1);
		CAN1_SendMesg(ID4, 8, Config_Kp);
	  delay_ms(1);

		CAN1_SendMesg(ID1, 8, Config_Ki);
	  delay_ms(1);
		CAN1_SendMesg(ID2, 8, Config_Ki);
	  delay_ms(1);
		CAN1_SendMesg(ID3, 8, Config_Ki);
	  delay_ms(1);
		CAN1_SendMesg(ID4, 8, Config_Ki);
	  delay_ms(1);

		CAN1_SendMesg(ID1, 8, Config_Kd);
	  delay_ms(1);
		CAN1_SendMesg(ID2, 8, Config_Kd);
	  delay_ms(1);
		CAN1_SendMesg(ID3, 8, Config_Kd);
	  delay_ms(1);
		CAN1_SendMesg(ID4, 8, Config_Kd);
	  delay_ms(1);
}

/****************************************************************************
���ܣ���챵��ָֹͣ��
��������
*/
void MOTOR_STOP(void)
{
	  CAN1_SendMesg(ID1, 8, stopmotor);
	  delay_ms(1);	
	  CAN1_SendMesg(ID2, 8, stopmotor);
	  delay_ms(1);	
	  CAN1_SendMesg(ID3, 8, stopmotor);
	  delay_ms(1);		  
	  CAN1_SendMesg(ID4, 8, stopmotor);
	  delay_ms(1);	
}
/****************************************************************************
���ܣ�4����챵�����ϲ���ָ��
��������
*/
void EraseFault(void)
{
	  CAN1_SendMesg(ID1, 8, ResetMotorFault);
	  delay_ms(1);	
	  CAN1_SendMesg(ID2, 8, ResetMotorFault);
	  delay_ms(1);	
	  CAN1_SendMesg(ID3, 8, ResetMotorFault);
	  delay_ms(1);		  
	  CAN1_SendMesg(ID4, 8, ResetMotorFault);
	  delay_ms(1);	
}
/****************************************************************************
����: 1��������������ʧ�ܡ�����ʹ�ܵ��ָ��
������idΪ���Ե��ID��
*/
void SingleMotorDisable(uint32_t id)
{
	CAN1_SendMesg(id, 8, ResetMotorFault);
	delay_ms(1);	
	CAN1_SendMesg(id, 8, stopmotor);  // ʧ�ܵ��
	delay_ms(1);	
	
	
	
	CAN1_SendMesg(id, 8, speedmode);
	delay_ms(1);		
	CAN1_SendMesg(id, 8, upspeedtime);
	delay_ms(1);	
	CAN1_SendMesg(id, 8, goalspeed);
	delay_ms(1);	
	CAN1_SendMesg(id, 8, enablemotor);
	delay_ms(1);	
}
/****************************************************************************
���ܣ�CAN�����ж�
��������
��������ճɹ���1��1�ŵ����־flag1��2�ŵ����־flag2��3�ŵ����־flag3��4�ŵ����־flag4
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	int i2 = 0;
	
	// ��� ���󱻶��ж����� ��־λ����ʱ����
  if(CAN_GetITStatus(CANx, CAN_FLAG_EPV))
	{
		CAN_ClearITPendingBit(CANx, CAN_FLAG_EPV);
	}
	
	if((CAN_MessagePending(CANx, CAN_FIFO0) != 0)) //���FIFO0�����Ƿ�������
	{
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
		
		if((RxMessage.StdId==0x01) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag1 = 1; 					   //���ճɹ�
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //�������
					//Clear_RxMes(&RxMessage);
			}
			
			for(i2=0; i2<8; i2++)    
			{
				ReceiveBuff1[i2] = RxMessage.Data[i2];
			}			
		}
		else
		{
			flag1 = 0; 					   //����ʧ��
		}
		
		if((RxMessage.StdId==0x02) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag2 = 1; 	
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //�������
			}
			
			for(i2=0; i2<8; i2++)          
			{
				ReceiveBuff2[i2] = RxMessage.Data[i2];
			}			
		}
		else
		{
			flag2 = 0; 					   
		}
		
		if((RxMessage.StdId==0x03) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag3 = 1; 
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //�������
			}
			
			for(i2=0; i2<8; i2++)         
			{
				ReceiveBuff3[i2] = RxMessage.Data[i2];
			}			
		}
		else
		{
			flag3 = 0; 					 
		}
		if((RxMessage.StdId==0x04) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag4 = 1;
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //�������
			}
			
			for(i2=0; i2<8; i2++)         
			{
				ReceiveBuff4[i2] = RxMessage.Data[i2];
			}
		}
		else
		{
			flag4 = 0; 					
		}

	}
	
}
/****************************************************************************
// ���ܣ�1��������ϼ����ָ�����
// �������� 
*/
void SingleMotorDeal(void)
{	
	if(flag1 == 1)
	{
		if( (RxMessage.Data[1] == 0xFE) && (RxMessage.Data[4] >= (OVERCURRENT/256))&& (RxMessage.Data[5] >= (OVERCURRENT%256)) )
		{
			goalspeed[6] = RxMessage.Data[6];
		  goalspeed[7] = RxMessage.Data[7];
			SingleMotorDisable(ID1);
		}		
		flag1 = 0;
	}
	if(flag2 == 1)
	{
		if( (RxMessage.Data[1] == 0xFE) && (RxMessage.Data[4] >= 0x04)&& (RxMessage.Data[5] >= 0xB0) )
		{
			goalspeed[6] = RxMessage.Data[6];
		  goalspeed[7] = RxMessage.Data[7];
			SingleMotorDisable(ID2);
			
		}		
		flag2 = 0;
	}
	if(flag3 == 1)
	{
		if( (RxMessage.Data[1] == 0xFE) && (RxMessage.Data[4] >= 0x04)&& (RxMessage.Data[5] >= 0xB0) )
		{
			goalspeed[6] = RxMessage.Data[6];
		  goalspeed[7] = RxMessage.Data[7];
			SingleMotorDisable(ID3);
			
		}		
		flag3 = 0;
	}
	if(flag4 == 1)
	{
		if( (RxMessage.Data[1] == 0xFE) && (RxMessage.Data[4] >= 0x04)&& (RxMessage.Data[5] >= 0xB0) )
		{
			goalspeed[6] = RxMessage.Data[6];
		  goalspeed[7] = RxMessage.Data[7];
			SingleMotorDisable(ID4);
			
		}		
		flag4 = 0;
	}
}


/****************************************************************************
���ܣ��ѵ���ٶȺͽ��ٶ��ϴ�������վ�����ReadData()����ʹ��
��������
�����SpeedFeedBack_Structure.velocity - �������ٶ�
      SpeedFeedBack_Structure.angular - �����ؽ��ٶ�
*/
void RTvelandang(void)
{
	float GroundStationVel = 0;  //��ȡʵʱ�ٶ�
  float GroundStationAng = 0;  //��ȡʵʱ���ٶ�
	float ThreeRTvel,ThreeRPM,FourRTvel,FourRPM;
	int16_t SpeedValue;
	
	CAN_ITConfig(CANx, CAN_IT_FMP0, DISABLE);  

	SpeedValue = (((uint16_t)(0x00FF) & ReceiveBuff3[6])<<8) + ReceiveBuff3[7];
	ThreeRPM = (float)(SpeedValue*3000.0/8192.0); // RPMֵ
	ThreeRTvel = (ThreeRPM*3.1415*0.195)/60;
	
	SpeedValue = (((uint16_t)(0x00FF) & ReceiveBuff4[6])<<8) + ReceiveBuff4[7];
	SpeedValue = (0-SpeedValue);
	FourRPM = (float)(SpeedValue*3000.0/8192.0); // RPMֵ
	FourRTvel = (FourRPM*3.1415*0.195)/60;
	
	GroundStationVel = (FourRTvel + ThreeRTvel)/2;
	GroundStationAng = (FourRTvel - ThreeRTvel)/0.480;
	
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

	SpeedFeedBack_Structure.velocity = (int16_t)(GroundStationVel * 100);
	SpeedFeedBack_Structure.angular = (int16_t)(GroundStationAng * 100);
}



/****************************************************************************
���ܣ���MotorFault=1ʱ�����������ϲ�����ʹ�ܵ��
��������
�������
*/
void ClearFault_EnableAgain(void)
{
	EraseFault();  // ���������ϰ�
	//delay_ms(300);
	Wheel_MOTOR_Init();
	MotorFault = 0;
	flag1 = 0;
	flag2 = 0;
	flag3 = 0;
	flag4 = 0;
}

/****************************************************************************
���ܣ���ʼ��CAN�������������
��������
�������
*/
void Motor_CAN_Init(void)
{
	CAN1_Config();
	CAN1_Config16BitFilter();
	Clear_RxMes(&RxMessage);
	Wheel_MOTOR_Init();

}

/****************************************************************************/



#endif  //OLD_WHEEL_MOTOR







































































