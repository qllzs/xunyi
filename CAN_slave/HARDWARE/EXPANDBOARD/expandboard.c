#include "expandboard.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)
uint16_t ADC_ConvertedValue[8];


void DigtalIOOutInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCCDigtalOUTGPIOClockCmd,ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = D1GPIOPIN|D2GPIOPIN|D3GPIOPIN|D4GPIOPIN|D8GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(DIGTALUPGPIOPORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = D5GPIOPIN|D6GPIOPIN|D7GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(DIGTALDOWNGPIOPORT, &GPIO_InitStructure);
	
	DigtalOutPutAllLow(); //全部输出低电平
}

void DigtalIOINInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_DIGTALIN1to3GPIOClockCmd|RCC_DIGTALIN4GPIOClockCmd,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = DIN1GPIOPIN|DIN2GPIOPIN|DIN3GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //浮空输入
	GPIO_Init(DIGTALIN1to3GPIOPORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = DIN4GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //浮空输入
	GPIO_Init(DIGTALIN4GPIOPORT, &GPIO_InitStructure);
}

void RGBIOUTInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_RGBGPIOClockCmd,ENABLE);	 //使能PB
	
	GPIO_InitStructure.GPIO_Pin = BGPIOPIN|RGPIOPIN|GGPIOPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	GPIO_Init(RGBGPIOPORT, &GPIO_InitStructure);			//根据设定参数初始化
	
	GPIO_ResetBits(RGBGPIOPORT,RGPIOPIN); //LED输出低电平
	GPIO_ResetBits(RGBGPIOPORT,BGPIOPIN);
	GPIO_ResetBits(RGBGPIOPORT,GGPIOPIN);	
}

void DigtalOutHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_SetBits(GPIOx,GPIO_Pin);
}

void DigtalOutLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	GPIO_ResetBits(GPIOx,GPIO_Pin);
}

void DigtalOutPutAllLow(void)
{
	GPIO_ResetBits(DIGTALUPGPIOPORT,D1GPIOPIN);
	GPIO_ResetBits(DIGTALUPGPIOPORT,D2GPIOPIN);
	GPIO_ResetBits(DIGTALUPGPIOPORT,D3GPIOPIN);
	GPIO_ResetBits(DIGTALUPGPIOPORT,D4GPIOPIN);
	GPIO_ResetBits(DIGTALUPGPIOPORT,D8GPIOPIN);
	
	GPIO_ResetBits(DIGTALDOWNGPIOPORT,D5GPIOPIN);
	GPIO_ResetBits(DIGTALDOWNGPIOPORT,D6GPIOPIN);
	GPIO_ResetBits(DIGTALDOWNGPIOPORT,D7GPIOPIN);
}

void HBridgePwmInit(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;
	GPIO_InitTypeDef          	GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_EXPANDPWMGPIOClockCmd,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_EXPANDPWMClockCmd,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 		= HBridgeOUTGPIOPIN; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(HBridgeOUTGPIOPORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin 		= EXPANDPWMGPIOPIN; 
	GPIO_InitStructure.GPIO_Speed 		= GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_AF_PP; 
	GPIO_Init(EXPANDPWMGPIOPORT, &GPIO_InitStructure);
	
	//TIM配置
	TIM_DeInit(TIMEXPANDPWM);
	TIM_TimeBaseStructure.TIM_Period	= 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler	= 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter	= 0; 
	TIM_TimeBaseInit(TIMEXPANDPWM,&TIM_TimeBaseStructure);
	
	//PWM1配置
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//当计数值小于设定值时输出低电平，故输入值越大转速越快
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Disable;//输出比较使能
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//装入比较器的值
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//起始低电平
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//空闲的时候输出低电平，TIM1有效
	TIM_OCInitStructure.TIM_OCNIdleState= 	TIM_OCIdleState_Reset;
	TIMEXPANDPWMOCInit(TIMEXPANDPWM,&TIM_OCInitStructure);
	
	TIMEXPANDPWMOCPreloadConfig(TIMEXPANDPWM,TIM_OCPreload_Enable);
	TIM_Cmd(TIMEXPANDPWM, ENABLE);  
	TIM_CtrlPWMOutputs(TIMEXPANDPWM, ENABLE);
	TIMEXPANDPWM->CCR3	= HBridgeDutyClc;
}

void IODigtalOut(void)
{
	DigtalIOOutInit();
	RGBIOUTInit();
	HBridgePwmInit();
	DigtalIOINInit();
}

void HBridgeDriveFwd(void)
{
	GPIO_SetBits(HBridgeOUTGPIOPORT,HBridgeOUTGPIOPIN);
}
void HBridgeDriveRev(void)
{
	GPIO_ResetBits(HBridgeOUTGPIOPORT,HBridgeOUTGPIOPIN);
}

void DigtalOutControl(uint8_t Channel)
{
	uint8_t	i=0;
	uint8_t	ChByte=0;
	
	ChByte	=	Channel;
	for(i=0;i<8;i++)
	{
		if (ChByte & 0x01)
		{
			switch(i)
			{
				case 0:
					DigtalOutHigh(DIGTALUPGPIOPORT,D1GPIOPIN);
				break;
				case 1:
					DigtalOutHigh(DIGTALUPGPIOPORT,D2GPIOPIN);
				break;
				case 2:
					DigtalOutHigh(DIGTALUPGPIOPORT,D3GPIOPIN);
				break;
				case 3:
					DigtalOutHigh(DIGTALUPGPIOPORT,D4GPIOPIN);
				break;
				case 4:
					DigtalOutHigh(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 5:
					DigtalOutHigh(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 6:
					DigtalOutHigh(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 7:
					DigtalOutHigh(DIGTALUPGPIOPORT,D8GPIOPIN);
				break;
			}
		}
		else
		{
			switch(i)
			{
				case 0:
					DigtalOutLow(DIGTALUPGPIOPORT,D1GPIOPIN);
				break;
				case 1:
					DigtalOutLow(DIGTALUPGPIOPORT,D2GPIOPIN);
				break;
				case 2:
					DigtalOutLow(DIGTALUPGPIOPORT,D3GPIOPIN);
				break;
				case 3:
					DigtalOutLow(DIGTALUPGPIOPORT,D4GPIOPIN);
				break;
				case 4:
					DigtalOutLow(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 5:
					DigtalOutLow(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 6:
					DigtalOutLow(DIGTALDOWNGPIOPORT,D5GPIOPIN);
				break;
				case 7:
					DigtalOutLow(DIGTALUPGPIOPORT,D8GPIOPIN);
				break;
			}
		}
		ChByte >>= 1;
	}
}

uint8_t DigtalINControl(uint8_t Channel)
{
	switch(Channel)
	{
		case 1:
			return GPIO_ReadInputDataBit(DIGTALIN1to3GPIOPORT,DIN1GPIOPIN);
		case 2:
			return GPIO_ReadInputDataBit(DIGTALIN1to3GPIOPORT,DIN2GPIOPIN);
		case 3:
			return GPIO_ReadInputDataBit(DIGTALIN1to3GPIOPORT,DIN3GPIOPIN);
		case 4:
			return GPIO_ReadInputDataBit(DIGTALIN4GPIOPORT,DIN4GPIOPIN);
		default:
			return 0;
	}
}

//模拟输入初始化
void ANALOG_IN_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_ANALOGIN, ENABLE);

	GPIO_InitStructure.GPIO_Pin = ADC1GPIO | ADC2GPIO | ADC3GPIO | ADC4GPIO | VOA1GPIO | VOA2GPIO | VOA3GPIO | VOA4GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ANALOGINPOART, &GPIO_InitStructure);	
	
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址固定
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
 // ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 //扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 	//要转换的通道数目
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的6分频，即12MHz,ADC频率最高不能超过14MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	/*配置ADC1的通道*/ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_55Cycles5);

	
	ADC_DMACmd(ADC1, ENABLE);  //使能DMA
	ADC_Cmd(ADC1, ENABLE);	 //使能ADC1  
	ADC_ResetCalibration(ADC1);  //复位校准寄存器
	while(ADC_GetResetCalibrationStatus(ADC1)); 	//等待校准寄存器复位完成
	ADC_StartCalibration(ADC1);  	//ADC校准
	while(ADC_GetCalibrationStatus(ADC1));  //等待校准完成
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//由于没有采用外部触发，所以使用软件触发ADC转换

}

void GetAnalogVolageIN(uint16_t* volagebuff)
{
	int i;
	for(i=0; i < 4; i++)
	{
		volagebuff[i] =  ADC_ConvertedValue[i];
	}
	
}

void GetAnalogCurrentIN(uint16_t* currentbuff)
{
	int j;
	for(j=4; j < 8; j++)
	{
		currentbuff[j] =  ADC_ConvertedValue[j];
	}
}
	


