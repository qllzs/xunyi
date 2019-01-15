
#include "bsp_OldMotor_CAN.h"
#ifdef OLD_WHEEL_MOTOR
uint8_t MotorFault=0;  // 电机故障标志

uint32_t flag1 = 0;		 //用于标志是否接收到数据，在中断函数中赋值
uint32_t flag2 = 0;	
uint32_t flag3 = 0;	
uint32_t flag4 = 0;

uint8_t ReceiveBuff1[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff2[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff3[8] = {0,0,0,0,0,0,0,0};
uint8_t ReceiveBuff4[8] = {0,0,0,0,0,0,0,0};

uint8_t speedmode[8]={0x00, 0xFA, 0x00, 0x19, 0x00, 0x00, 0x00, 0x2F}; //设置速度模式
uint8_t upspeedtime[8]={0x00, 0xFA, 0x00, 0x13, 0x00, 0x00, 0x0A, 0x0B}; //设置加减速时间
uint8_t goalspeedzero[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
uint8_t goalspeed[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
uint8_t enablemotor[8]={0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F}; //使能电机
uint8_t stopmotor[8]={0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F};  //stop
uint8_t ResetMotorFault[8] = {0x00, 0xFA, 0x00, 0x15, 0x00, 0x00, 0x00, 0x7F};  // 复位故障

//改变速度

uint8_t Config_Ki[8]={0x00, 0xFA, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00}; 
uint8_t Config_Kp[8]={0x00, 0xFA, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00}; 
uint8_t Config_Kd[8]={0x00, 0xFA, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00}; 



//static CanTxMsg TxMessage;			     //发送缓冲区
static CanRxMsg RxMessage;				   //接收缓冲区

/*
故障类型声明
02 过流： 00 FF 00 EB 02 02 02 02
04 过压： 00 FF 00 EB 04 04 04 04
08 编码器故障： 00 FF 00 EB 08 08 08 08
10 过热： 00 FF 00 EB 10 10 10 10
20 欠压： 00 FF 00 EB 20 20 20 20
40 过载状态： 00 FF 00 EB 40 40 40 40
01 运行状态： 00 FF 00 EB 01 01 01 01
00 内部报警代码：在内部控制模式下报警代码加上 00
80 外部控制代码：在外部控制模式下报警代码加上 80
*/

/****************************************************************************
功能：CAN 的 NVIC 配置
参数：无
*/
static void CAN1_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  // USB_LP_CAN1_RX0_IRQHandler
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// FIFO0消息挂号中断允许;
	CAN_ITConfig(CAN1, CAN_IT_FMP0 | CAN_IT_EPV, ENABLE);   	
	// CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);   	
	
	
}

/****************************************************************************
功能：CAN1引脚配置及CAN模式初始化
参数：无
*/
static void CAN1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	
	/* GPIO 时钟及引脚源 */
	RCC_APB2PeriphClockCmd(CAN_GPIO_AF_CLK | CAN_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出
	GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;  // GPIO_Pin_12; //PA12
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN;         //GPIO_Pin_11; //PA11
	GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);

	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;    //失能时间触发模式
	CAN_InitStructure.CAN_ABOM = ENABLE;    //失能自动离线管理
	CAN_InitStructure.CAN_AWUM = ENABLE;    //失能睡眠模式通过软件唤醒
	CAN_InitStructure.CAN_NART = DISABLE;    //失能非自动重传输模式（也就是会自动重传输）
	CAN_InitStructure.CAN_RFLM = DISABLE;    //失能接收FIFO锁定模式，新数据会覆盖旧数据
	CAN_InitStructure.CAN_TXFP = DISABLE;    //优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;       //有普通模式和拓展模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq; //重新同步跳跃宽度 1 个时间单位
  /* 波特率设置, 当APB1的时钟频率是36MHZ的时候。 波特率的公式为： */
  /* 波特率(Kpbs) = 36M / ((CAN_BS1 + CAN_BS2 + 1) *  CAN_Prescaler) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //时间段 1 为8 个时间单位 
	CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq; //时间段 2 为7 个时间单位
	CAN_InitStructure.CAN_Prescaler = 8;	 

	CAN_Init(CANx, &CAN_InitStructure);

	#ifdef CAN_RX0_INT_ENABLE
		CAN1_NVIC_Config();
	#endif    
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
	TxMessage.IDE = CAN_ID_STD;	    //拓展ID   
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
功能：CAN1筛选器配置 
参数：无
*/
static void CAN1_Config16BitFilter(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	uint16_t mask = 0xFFF8;

	CAN_FilterInitStructure.CAN_FilterNumber = 1;	                  //过滤器1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; //ID模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;	// 寄存器组设置为16位 

	CAN_FilterInitStructure.CAN_FilterIdHigh= ((u16)0x03<<5);		  //要筛选的ID高位 
	//CAN_FilterInitStructure.CAN_FilterIdLow= (0X00<<5);
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= (mask<<5);			//匹配
	//CAN_FilterInitStructure.CAN_FilterMaskIdLow= (mask<<5);	

	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;  //使能过滤器1
	CAN_FilterInit(&CAN_FilterInitStructure);
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
/****************************************************************************
功能：轮毂电机初始化配置-速度模式，初始速度，电机使能
参数：无
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
		//设置电机的PID参数
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
功能：轮毂电机停止指令
参数：无
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
功能：4个轮毂电机故障擦除指令
参数：无
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
功能: 1个电机故障清除、失能、重新使能电机指令
参数：id为各自电机ID号
*/
void SingleMotorDisable(uint32_t id)
{
	CAN1_SendMesg(id, 8, ResetMotorFault);
	delay_ms(1);	
	CAN1_SendMesg(id, 8, stopmotor);  // 失能电机
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
功能：CAN接收中断
参数：无
输出：接收成功置1，1号电机标志flag1；2号电机标志flag2；3号电机标志flag3；4号电机标志flag4
*/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	int i2 = 0;
	
	// 检测 错误被动中断屏蔽 标志位并及时清零
  if(CAN_GetITStatus(CANx, CAN_FLAG_EPV))
	{
		CAN_ClearITPendingBit(CANx, CAN_FLAG_EPV);
	}
	
	if((CAN_MessagePending(CANx, CAN_FIFO0) != 0)) //检查FIFO0里面是否有数据
	{
		CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
		
		if((RxMessage.StdId==0x01) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag1 = 1; 					   //接收成功
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //电机故障
					//Clear_RxMes(&RxMessage);
			}
			
			for(i2=0; i2<8; i2++)    
			{
				ReceiveBuff1[i2] = RxMessage.Data[i2];
			}			
		}
		else
		{
			flag1 = 0; 					   //接收失败
		}
		
		if((RxMessage.StdId==0x02) && (RxMessage.IDE==CAN_ID_STD) && (RxMessage.DLC==8) )
		{
			flag2 = 1; 	
			
			if( (RxMessage.Data[7]&0xFF) && (RxMessage.Data[3] == 0xEB) && (RxMessage.Data[1] ==0xFF ) )
			{
					MotorFault = 1;   //电机故障
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
					MotorFault = 1;   //电机故障
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
					MotorFault = 1;   //电机故障
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
// 功能：1个电机故障检查与恢复处理
// 参数：无 
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
功能：把电机速度和角速度上传至地面站，配合ReadData()函数使用
参数：无
输出：SpeedFeedBack_Structure.velocity - 待返回速度
      SpeedFeedBack_Structure.angular - 待返回角速度
*/
void RTvelandang(void)
{
	float GroundStationVel = 0;  //读取实时速度
  float GroundStationAng = 0;  //读取实时角速度
	float ThreeRTvel,ThreeRPM,FourRTvel,FourRPM;
	int16_t SpeedValue;
	
	CAN_ITConfig(CANx, CAN_IT_FMP0, DISABLE);  

	SpeedValue = (((uint16_t)(0x00FF) & ReceiveBuff3[6])<<8) + ReceiveBuff3[7];
	ThreeRPM = (float)(SpeedValue*3000.0/8192.0); // RPM值
	ThreeRTvel = (ThreeRPM*3.1415*0.195)/60;
	
	SpeedValue = (((uint16_t)(0x00FF) & ReceiveBuff4[6])<<8) + ReceiveBuff4[7];
	SpeedValue = (0-SpeedValue);
	FourRPM = (float)(SpeedValue*3000.0/8192.0); // RPM值
	FourRTvel = (FourRPM*3.1415*0.195)/60;
	
	GroundStationVel = (FourRTvel + ThreeRTvel)/2;
	GroundStationAng = (FourRTvel - ThreeRTvel)/0.480;
	
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

	SpeedFeedBack_Structure.velocity = (int16_t)(GroundStationVel * 100);
	SpeedFeedBack_Structure.angular = (int16_t)(GroundStationAng * 100);
}



/****************************************************************************
功能：当MotorFault=1时，清除电机故障并重新使能电机
参数：无
输出：无
*/
void ClearFault_EnableAgain(void)
{
	EraseFault();  // 擦除所有障碍
	//delay_ms(300);
	Wheel_MOTOR_Init();
	MotorFault = 0;
	flag1 = 0;
	flag2 = 0;
	flag3 = 0;
	flag4 = 0;
}

/****************************************************************************
功能：初始化CAN及电机驱动配置
参数：无
输出：无
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







































































