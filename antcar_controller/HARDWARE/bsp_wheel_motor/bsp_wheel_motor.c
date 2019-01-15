#include "bsp_wheel_motor.h"

volatile uint8_t MotorRecFlag=0;
uint8_t Rx_Buff[DMA_Rx_BUFFER_SIZE] = {0};
uint8_t Tx_Buff[DMA_Tx_BUFFER_SIZE] = {0};
int16_t max_forward_speed = 120;//单位为：cm/s
int16_t max_backward_speed = -120;
int16_t VelOfCar = 0, AngOfCar = 0;//整个系统的速度和角速度兼容上层控制盒自动巡线功能
 /**
  * @brief  配置嵌套向量中断控制器NVIC
  * @param  无
  * @retval 无
  */
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = Motor_USART_IRQ;
  /* 抢断优先级*/
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}
/**
  * @brief  USART DMA 配置,工作参数配置
  * @param  无
  * @retval 无
  */
static void WheelMotor_USART_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA_USARTx_Motor, ENABLE);
	
//DMA接收通道初始化
	DMA_InitStructure.DMA_BufferSize = DMA_Rx_BUFFER_SIZE;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Rx_Buff;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&Motor_USARTx->DR);
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	
	DMA_Init(Motor_USARTx_DMA_Rx_Channel, &DMA_InitStructure);
	DMA_Cmd(Motor_USARTx_DMA_Rx_Channel,ENABLE);
//DMA发送通道初始	
	DMA_InitStructure.DMA_BufferSize = DMA_Tx_BUFFER_SIZE;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Tx_Buff;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&Motor_USARTx->DR);
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	
	DMA_Init(Motor_USARTx_DMA_Tx_Channel, &DMA_InitStructure);
	
	DMA_Cmd(Motor_USARTx_DMA_Tx_Channel,DISABLE);
}
 /**
  * @brief  USART工作参数配置
  * @param  无
  * @retval 无
  */
static void bsp_WheelMotor_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// 打开串口GPIO的时钟
	Motor_USART_GPIO_APBxClkCmd(Motor_USART_GPIO_CLK, ENABLE);
	
	// 打开串口外设的时钟
	Motor_USART_APBxClkCmd(Motor_USART_CLK, ENABLE);

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = Motor_USART_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(Motor_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  // 将USART Rx的GPIO配置为浮空输入模式
	GPIO_InitStructure.GPIO_Pin = Motor_USART_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(Motor_USART_RX_GPIO_PORT, &GPIO_InitStructure);
	
	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate = Motor_USART_BAUDRATE;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(Motor_USARTx, &USART_InitStructure);
	
	// 串口中断优先级配置
	NVIC_Configuration();
	USART_ITConfig(Motor_USARTx, USART_IT_IDLE, ENABLE );
	USART_Cmd(Motor_USARTx, ENABLE);	
	WheelMotor_USART_DMA_Config();
	USART_DMACmd(Motor_USARTx,USART_DMAReq_Rx,ENABLE);	    
}

static void InvertUint8(unsigned char *dBuf,unsigned char *srcBuf)
{
	int i;
	unsigned char tmp[4];
	tmp[0] = 0;
	for(i=0;i< 8;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(7-i);
	}
	dBuf[0] = tmp[0];
	
}
static void InvertUint16(unsigned short *dBuf,unsigned short *srcBuf)
{
	int i;
	unsigned short tmp[4];
	tmp[0] = 0;
	for(i=0;i< 16;i++)
	{
		if(srcBuf[0]& (1 << i))
		tmp[0]|=1<<(15 - i);
	}
	dBuf[0] = tmp[0];
}
/*
 *@brief：CRC16产生的函数
 *@param：pushMsg-输入信息 usDataLen-数据长度
 *@retval：CRC16值
 */
unsigned short CRC16_MODBUS(unsigned char *puchMsg, unsigned int usDataLen)
{
  unsigned short wCRCin = 0xFFFF;
  unsigned short wCPoly = 0x8005;
  unsigned char wChar = 0;
  int i = 0;
  while (usDataLen--) 	
  {
        wChar = *(puchMsg++);
        InvertUint8(&wChar,&wChar);
        wCRCin ^= (wChar << 8);
        for(i = 0;i < 8;i++)
        {
          if(wCRCin & 0x8000)
            wCRCin = (wCRCin << 1) ^ wCPoly;
          else
            wCRCin = wCRCin << 1;
        }
  }
  InvertUint16(&wCRCin,&wCRCin);
  return (wCRCin) ;
}
/*
 *@brief: 写电机寄存器函数
 *@param：id-电机id编号,reg-寄存器地址
 *@retval：无
 */
static void Write_MotorReg_usart(uint8_t id, uint16_t reg, uint16_t val)
{
	uint16_t crc16 = 0;
	Tx_Buff[0] = id;
	Tx_Buff[1] = W_S_REG;
	Tx_Buff[2] = (reg >> 8)&0xff;
	Tx_Buff[3] = (reg >> 0)&0xff;
	Tx_Buff[4] = (val >> 8)&0xff;
	Tx_Buff[5] = (val >> 0)&0xff;
	crc16 = CRC16_MODBUS(Tx_Buff, 6);
	Tx_Buff[6] = (crc16 >> 0)&0xff;
	Tx_Buff[7] = (crc16 >> 8)&0xff;
	USART_DMACmd(Motor_USARTx,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(Motor_USARTx_DMA_Tx_Channel,DISABLE);
	DMA_SetCurrDataCounter(Motor_USARTx_DMA_Tx_Channel,DMA_Tx_BUFFER_SIZE);
	DMA_Cmd(Motor_USARTx_DMA_Tx_Channel,ENABLE);
	
	while(!DMA_GetFlagStatus(Motor_USARTx_DMA_FLAG_TC));
	DMA_ClearFlag(DMA1_FLAG_TC4);
	while(!USART_GetFlagStatus(Motor_USARTx, USART_FLAG_TC));
	USART_ClearFlag(Motor_USARTx, USART_FLAG_TC);
	delay_ms(1);
}
/*
 *@brief：读电机的寄存器函数
 *@param：id-电机id编号,reg-寄存器地址,len-数据长度,单位为字
 *@retval：
 *
 */
static uint16_t Read_MotorReg_usart(uint8_t id, uint16_t reg, uint16_t len)
{
	uint16_t crc16 = 0, RegVal = 0;
	uint32_t i = MOTOR_TIMEOUT;
	Tx_Buff[0] = id;
	Tx_Buff[1] = R_REG;
	Tx_Buff[2] = (reg >> 8)&0xff;
	Tx_Buff[3] = (reg >> 0)&0xff;
	Tx_Buff[4] = (len >> 8)&0xff;
	Tx_Buff[5] = (len >> 0)&0xff;
	crc16 = CRC16_MODBUS(Tx_Buff, 6);
	Tx_Buff[6] = (crc16 >> 0)&0xff;
	Tx_Buff[7] = (crc16 >> 8)&0xff;
	USART_DMACmd(Motor_USARTx,USART_DMAReq_Tx,ENABLE);
	DMA_Cmd(Motor_USARTx_DMA_Tx_Channel,DISABLE);
	DMA_SetCurrDataCounter(Motor_USARTx_DMA_Tx_Channel,DMA_Tx_BUFFER_SIZE);
	DMA_Cmd(Motor_USARTx_DMA_Tx_Channel,ENABLE);
	MotorRecFlag = 0;
	while(!DMA_GetFlagStatus(Motor_USARTx_DMA_FLAG_TC));
	DMA_ClearFlag(Motor_USARTx_DMA_FLAG_TC);
	while(!USART_GetFlagStatus(Motor_USARTx, USART_FLAG_TC));
	USART_ClearFlag(Motor_USARTx, USART_FLAG_TC);
	while(i--)
	{
		if(i ==0 )
		{
			//没有收到数据,超时处理
			
			break;
		}
		if(MotorRecFlag)
		{
			MotorRecFlag = 0;
			RegVal = (uint16_t)((Rx_Buff[3] << 8) | Rx_Buff[4]);
			break;
		}
	}
	//delay_ms(20);
	return RegVal;	
}

/*
 *@brief：轮毂电机驱动器的初始化
 *@param：无
 *@retval：1：初始化成功   0：初始化失败
 */
uint8_t bsp_WheelMotor_init(void)
{
	uint16_t  Kp_r = 0, Kp_l = 0, Ki_r = 0, Ki_l = 0;
	bsp_WheelMotor_USART_Config();
	//延时等待驱动器初始化完成
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	//控制模式设为内部速度模式
	Write_MotorReg_usart(ID_MOTOR_L, CON_MOD_REG, 0);
	Write_MotorReg_usart(ID_MOTOR_R, CON_MOD_REG, 0);
	//速度初始化为0
	Write_MotorReg_usart(ID_MOTOR_L, INTER_SPEED_REG, 0);
	Write_MotorReg_usart(ID_MOTOR_R, INTER_SPEED_REG, 0);
	//启动电机
	Write_MotorReg_usart(ID_MOTOR_L, IO_TOGGLE_REG, 0x01);
	Write_MotorReg_usart(ID_MOTOR_R, IO_TOGGLE_REG, 0x01);
	//读取PID参数
	Kp_r = Read_MotorReg_usart(ID_MOTOR_R, KP_SPEED_REG, 1);
	Ki_r = Read_MotorReg_usart(ID_MOTOR_R, KI_SPEED_REG, 1);
	Kp_l = Read_MotorReg_usart(ID_MOTOR_L, KP_SPEED_REG, 1);
	Ki_l = Read_MotorReg_usart(ID_MOTOR_L, KI_SPEED_REG, 1);
	//更改PID参数
//	Kp_r = x;
//	Ki_r = x;
//	Kp_l = x;
//	Ki_l = x;
	//写入PID参数	
	Write_MotorReg_usart(ID_MOTOR_R, KP_SPEED_REG, Kp_r);
	Write_MotorReg_usart(ID_MOTOR_R, KI_SPEED_REG, Ki_r);
	Write_MotorReg_usart(ID_MOTOR_L, KP_SPEED_REG, Kp_l);
	Write_MotorReg_usart(ID_MOTOR_L, KI_SPEED_REG, Ki_l);
	//参数保存	
	Write_MotorReg_usart(ID_MOTOR_R, PARA_SAVE, 1);	
	Write_MotorReg_usart(ID_MOTOR_L, PARA_SAVE, 1);
	return 1;
}
/****************************************************************************
功能：将小车的速度和角速度转化为左右电机的转速
参数：传入小车的设定速度和设定角速度单位分别为0.01m/s 和 0.01rad/s 传出左右电机的转速单位为rpm
输出：无
*/
static void CarSpeed2rpm(int16_t VelCar, int16_t AngCar,int16_t *Speed_RPM_LEFT,int16_t *Speed_RPM_RIGHT)
{
	#ifdef METEC_MOTOR
	float vr_mps = 0, vl_mps = 0;
	float vel = 0, ang = 0;
	vel = (float)VelCar/100;
	ang = (float)AngCar/100;
	
	#ifdef Ver2
	if(ModeReg == 1)
	{
		vr_mps = (2*vel+ang*D_Car/1000)/2.0;//分别计算左右轮的速度m/s）	
		vl_mps = (2*vel-ang*D_Car/1000)/2.0;
	}
	else
	{
		vr_mps = (2*vel-ang*D_Car/1000)/2.0;//分别计算左右轮的速度m/s）
		vl_mps = (2*vel+ang*D_Car/1000)/2.0;
	}
	#else
	vr_mps = (2*vel+ang*D_Car/1000)/2.0;//分别计算左右轮的速度m/s）
	vl_mps = (2*vel-ang*D_Car/1000)/2.0;
	#endif
	*Speed_RPM_RIGHT = 10*(60*vr_mps/(3.142*D_Motor/1000.0));//将m/s转化为0.1rpm
	*Speed_RPM_LEFT = 10*(60*vl_mps/(3.142*D_Motor/1000.0));
	#endif //METEC_MOTOR || MOTOR_485
  
	#ifdef OLD_WHEEL_MOTOR
	float velocityright, velocityleft; // 右左电机速度
	float right0,left0; //RPM value
	
	velocityright = VelCar/100.0 + 0.5*0.480*AngCar/100.0; //wheelbase is 0.480
	velocityleft = VelCar/100.0 -0.5*0.48*AngCar/100.0;
	
	right0 = (float)((60*velocityright)/(3.14*0.195)); 
	*Speed_RPM_RIGHT = (int16_t)(right0*8192.0/3000.0);
	left0 = (float)((60*velocityleft)/(3.14*0.195)); 
	*Speed_RPM_LEFT = (int16_t)(left0*8192.0/3000.0);
	#endif  //OLD_WHEEL_MOTOR
}

/****************************************************************************
 @brief ：设置电机的转速
 @param ：Speed_RPM_L - 左侧电机的转速     Speed_RPM_R - 右侧电机的转速 单位为0.1RPM      
 @retvel：0: 事变     1：成功
*/

 int16_t Speed_RPM_Left_L = 0, Speed_RPM_Left_H;
 int16_t Speed_RPM_Right_L = 0, Speed_RPM_Right_H;
 uint8_t CANErroStatus=0,CANErroCnt=0;
static uint8_t MotorSpeedSet(int16_t Speed_RPM_Set_L ,int16_t Speed_RPM_Set_R )
{
	#ifdef METEC_MOTOR  //莫泰克电机
	
	
	Speed_RPM_Set_L = Speed_RPM_Set_L * RATIO_TRANSMISSION;
	Speed_RPM_Set_R = Speed_RPM_Set_R * RATIO_TRANSMISSION;
	
	//Speed_RPM_Set_L = -Speed_RPM_Set_L;
	#ifndef Ver2
		Speed_RPM_Set_R = -Speed_RPM_Set_R;
	#else
		Speed_RPM_Set_L = -Speed_RPM_Set_L;
	#endif
	Speed_RPM_Left_L = (((Speed_RPM_Set_L) >> 0) & 0xffff);
	Speed_RPM_Left_H = (((Speed_RPM_Set_L) >> 16) & 0xffff);
	
	Speed_RPM_Right_L = (((Speed_RPM_Set_R) >> 0) & 0xffff);
	Speed_RPM_Right_H = (((Speed_RPM_Set_R) >> 16) & 0xffff);
	
	if((Ang_Set !=0) && (Vel_Set ==0) && (((Speed_RPM_Set_L > 0) && (Speed_RPM_Set_R >0)) || ((Speed_RPM_Set_L < 0) && (Speed_RPM_Set_R < 0))))
	{
//		sprintf(WriteBuffer,"\n执行数据错误");
//		fileWrite(WriteBuffer,"0:上层发来数据与执行不符.txt");	
	}

	SendMessage2MotorDriver(MOTOR_L,PRIM_SETVELOCITY,Speed_RPM_Left_L,Speed_RPM_Left_H);	
	SendMessage2MotorDriver(MOTOR_R,PRIM_SETVELOCITY,Speed_RPM_Right_L,Speed_RPM_Right_H);
	CANErroStatus = CAN_GetLastErrorCode(CAN1);
	CANErroCnt = CAN_GetReceiveErrorCounter(CAN1);
	#endif
	
	#ifdef MOTOR_485  //485电机
/**********根据左右轮的速度的正负进行寄存器方向位的设置***********/
	Speed_RPM_Set_L = Speed_RPM_Set_L * RATIO_TRANSMISSION;
	Speed_RPM_Set_R = Speed_RPM_Set_R * RATIO_TRANSMISSION;
	if(Speed_RPM_Set_L >= 0)
	{
		Write_MotorReg_usart(ID_MOTOR_L, IO_TOGGLE_REG, 0x1);
	}
	else
	{
		Speed_RPM_L = 0 - Speed_RPM_L;
		Write_MotorReg_usart(ID_MOTOR_L, IO_TOGGLE_REG, 0x3);	
	}
	if(Speed_RPM_R >= 0)
	{
		Write_MotorReg_usart(ID_MOTOR_R, IO_TOGGLE_REG, 0x3);
	}
	else
	{
		Write_MotorReg_usart(ID_MOTOR_R, IO_TOGGLE_REG, 0x1);	
		Speed_RPM_R = 0 - Speed_RPM_R;
	}	
	Write_MotorReg_usart(ID_MOTOR_L, INTER_SPEED_REG, Speed_RPM_Set_L);
	Write_MotorReg_usart(ID_MOTOR_R, INTER_SPEED_REG, Speed_RPM_Set_R);		
	#endif
	#ifdef OLD_WHEEL_MOTOR //如果定义了旧的轮毂电机
	
	uint8_t speedrightbuffer[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
	uint8_t speedleftbuffer[8]={0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00}; 
	
	speedrightbuffer[6] = ((uint8_t)(0xFF) & (0-Speed_RPM_Set_R)>>8);
	speedrightbuffer[7] = ((uint8_t)(0xFF) & (0-Speed_RPM_Set_R));
	
	speedleftbuffer[6] = (uint8_t)(0xFF) & (Speed_RPM_Set_L)>>8;
	speedleftbuffer[7] = (uint8_t)(0xFF) & Speed_RPM_Set_L;
	
	CAN1_SendMesg(ID1, 8, speedleftbuffer);
	delay_ms(1);	
	CAN1_SendMesg(ID3, 8, speedleftbuffer);
	delay_ms(1);	
	CAN1_SendMesg(ID2, 8, speedrightbuffer);
	delay_ms(1);	
	CAN1_SendMesg(ID4, 8, speedrightbuffer);
	delay_ms(1);	
	
	
	#endif
/**********设置电机的左右轮速度***********/

	return 1;
}
/**
 *@brief ：通过滤波处理防止速度变化过快
 *@param ：
 *@retvel:
 */
static void SpeedFilter(int16_t *Vel, int16_t *Ang)
{
	
	static int16_t velocity_old = 0, velocity_new = 0, angular_old = 0, angular_new = 0;
	
	velocity_old = velocity_new;
	velocity_new = *Vel;
	angular_old = angular_new;
	angular_new = *Ang;
	if(fabs((velocity_new - velocity_old)) >= 10)
	{
		velocity_new = (velocity_new + velocity_old)/2;
	}
	if(fabs((angular_new - angular_old)) >= 10)
	{
		angular_new = (angular_new + angular_old)/2;
	}		
	*Vel = velocity_new;
	*Ang = angular_new;

	if((*Vel) > max_forward_speed)
	{
		*Vel = max_forward_speed;
	}
	if((*Vel) < max_backward_speed)
	{
		*Vel = max_backward_speed;
	}
}
/****************************************************************************
功能：设置小车的速度和角速度
参数：传入小车的设定速度和设定角速度单位分别为0.01m/s 和 0.01rad/s
输出：无
*/
int16_t Speed_RPM_L = 0, Speed_RPM_R = 0;
void CarSpeedSet(int16_t Vel_Set, int16_t Ang_Set)
{
	
	AntiColStatusCheck(&Vel_Set , &Ang_Set);
	SpeedFilter(&Vel_Set, &Ang_Set);
	CarSpeed2rpm(Vel_Set, Ang_Set,&Speed_RPM_L,&Speed_RPM_R);
	if(ModeReg == 1)
		ModeReg = 1;
	if(ModeReg == 0)
		ModeReg = 0;
	MotorSpeedSet(Speed_RPM_L ,Speed_RPM_R );
}
/*
 *@brief：读取单个电机速度函数
 *@param：id-电机的id编号
 *@reval：speed_feedback-返回的单个电机的实际速度大小，单位为rpm
 */
//static int16_t ReadSigleMotorSpeed(uint8_t id)
//{
//	#ifdef METEC_MOTOR
//	SendMessage2MotorDriver(MOTOR_L,PRIM_GETACTVELOCITY,0,0);	
//	SendMessage2MotorDriver(MOTOR_R,PRIM_GETACTVELOCITY,0,0);
//	#else
//	int16_t speed_feedback = 0 ,dir = 0;
//	/*******************************读取电机的速度大小*************************************************/
//	speed_feedback = Read_MotorReg_usart(id, CURRENT_SPEED_REG, 1);
//	/*******************************读取电机的转速方向*************************************************/
//	dir = Read_MotorReg_usart(id, IO_TOGGLE_REG, 1);
//	if(!(dir & (0x1 << 1)))//电机正转
//	{
//			speed_feedback = 0 - speed_feedback;
//	}
//	return speed_feedback;	
//	#endif
//return 0;
//}
/*
 *@brief：读取小车的速度
 *@param：待修改的小车速度和角速度Vel - 小车速度0.01m/s   Ang - 小车角速度0.01rad/s
 *@retval：none
 *@notice：ang>0 --左转弯     ang<0  --  右转弯
 *         小车电机编号从左到右分别是1 2
 */
void CarSpeedRead(int16_t* Vel, int16_t *Ang)
{
	
	#ifdef METEC_MOTOR
	SendMessage2MotorDriver(MOTOR_L,PRIM_GETACTVELOCITY,0,0);	
	SendMessage2MotorDriver(MOTOR_R,PRIM_GETACTVELOCITY,0,0);
	*Vel = (int16_t)(100*(float)(Speed_R_RPM + Speed_L_RPM)/600.0/RATIO_TRANSMISSION*0.5*3.14*D_Motor*0.001);
	
	*Ang = (int16_t )(100*(float)(Speed_R_RPM - Speed_L_RPM)/600.0/RATIO_TRANSMISSION*3.14*D_Motor*0.001/(D_Car*0.001));

	#endif
	#ifdef MOTOR_485
	int16_t Vel_F_R =0 ,Vel_F_L = 0;//反馈回来的左右轮速度，单位为rpm
	Vel_F_L = 0 - ReadSigleMotorSpeed(ID_MOTOR_L);//实际的转每分钟
  Vel_F_R = ReadSigleMotorSpeed(ID_MOTOR_R);//实际的转每分钟
	
	*Vel = (int16_t)(100*(float)(Vel_F_L + Vel_F_R)/60.0*RATIO_TRANSMISSION*0.5*3.14*D_Motor*0.001);
	
	*Ang = (int16_t )(100*(float)(Vel_F_R - Vel_F_L)/60.0*RATIO_TRANSMISSION*3.14*D_Motor*0.001/(D_Car*0.001));
	#endif
}
/**
	*@brief:单个电机的故障检测
	*@param：电机的id编号
	*@retval:1： 检测成功    0：检测失败
	*/
//static uint8_t SingleMotorFaultCheck(uint8_t id)
//{
//	uint16_t MotorAlarmFlag = 0;
//	MotorAlarmFlag = Read_MotorReg_usart(id, FAULT_STAT_REG, 1);
//	switch(MotorAlarmFlag)
//	{
//		case LOCKED_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 1;
//		}
//		case OVER_CURRENT_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 2;
//		}
//		case HAL_FAULT_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 3;
//		}
//		case VOL_DOWM_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 4;
//		}
//		case VOL_UP_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 5;
//		}
//		case SPEED_OVER_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 6;
//		}
//		case OVERLOAD_PRO_ALARM:
//		{
//			//Write_MotorReg_usart(id, ALARM_CLEAR, 0X01);
//			return 7;
//		}
//		default:
//		{	
//			return 0;
//		}
//	}
//}


/**
	*@brief:电机的故障检测
	*@param：电机的故障寄存器
	*@retval:1： 检测成功    0：检测失败
	*/
void CarMotorFaultCheck(volatile uint16_t * Fault_reg)
{
//	uint8_t Fault_num_L =0,Fault_num_R =0;
#ifdef METEC_MOTOR
	SendMessage2MotorDriver(MOTOR_L,PRIM_GETERROR,0,0);	
	SendMessage2MotorDriver(MOTOR_R,PRIM_GETERROR,0,0);
#endif
	
#ifdef MOTOR_485
	Fault_num_L = SingleMotorFaultCheck(ID_MOTOR_R);
	Fault_num_R = SingleMotorFaultCheck(ID_MOTOR_R);
	if(Fault_num_L)
	{
		*Fault_reg = (Fault_num_L << 0);
	}
	if(Fault_num_R)
	{
		*Fault_reg = (Fault_num_R << 4);
	}	
#endif

}


/**
	*@brief:USART1中断函数
	*@param：无
	*@retval:无
	*/
/*    回发：驱动器编号＋功能码＋字节数＋数据高８位＋数据低８位＋CRC16     */
void Motor_USART_IRQHandler(void)                	//串口中断服务程序
{
	uint8_t num=0;
	if(USART_GetITStatus(Motor_USARTx,USART_IT_IDLE) == SET)
	{
		num = Motor_USARTx->SR;
		num = Motor_USARTx->DR;
		num = num;
		DMA_Cmd(Motor_USARTx_DMA_Rx_Channel,DISABLE);
		num = DMA_Rx_BUFFER_SIZE - DMA_GetCurrDataCounter(Motor_USARTx_DMA_Rx_Channel);
		Motor_USARTx_DMA_Rx_Channel->CNDTR = DMA_Rx_BUFFER_SIZE;
		DMA_Cmd(Motor_USARTx_DMA_Rx_Channel,ENABLE);
		USART_DMACmd(Motor_USARTx,USART_DMAReq_Rx,ENABLE);
		if(CRC16_MODBUS(Rx_Buff, 3+Rx_Buff[2]) == (Rx_Buff[3+Rx_Buff[2]] | (Rx_Buff[3+Rx_Buff[2] + 1] << 8)))
		{
			MotorRecFlag = 1;
		}
		else
		{
			MotorRecFlag = 0;
		}	
	}
}




