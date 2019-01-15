#include "expandboard.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)
uint16_t ADC_ConvertedValue[8];


void DigtalIOOutInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCCDigtalOUTGPIOClockCmd,ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = D1GPIOPIN|D2GPIOPIN|D3GPIOPIN|D4GPIOPIN|D8GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(DIGTALUPGPIOPORT, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = D5GPIOPIN|D6GPIOPIN|D7GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(DIGTALDOWNGPIOPORT, &GPIO_InitStructure);
	
	DigtalOutPutAllLow(); //ȫ������͵�ƽ
}

void DigtalIOINInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_DIGTALIN1to3GPIOClockCmd|RCC_DIGTALIN4GPIOClockCmd,ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = DIN1GPIOPIN|DIN2GPIOPIN|DIN3GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //��������
	GPIO_Init(DIGTALIN1to3GPIOPORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = DIN4GPIOPIN;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 		 //��������
	GPIO_Init(DIGTALIN4GPIOPORT, &GPIO_InitStructure);
}

void RGBIOUTInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_RGBGPIOClockCmd,ENABLE);	 //ʹ��PB
	
	GPIO_InitStructure.GPIO_Pin = BGPIOPIN|RGPIOPIN|GGPIOPIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(RGBGPIOPORT, &GPIO_InitStructure);			//�����趨������ʼ��
	
	GPIO_ResetBits(RGBGPIOPORT,RGPIOPIN); //LED����͵�ƽ
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
	
	//TIM����
	TIM_DeInit(TIMEXPANDPWM);
	TIM_TimeBaseStructure.TIM_Period	= 1000-1;
	TIM_TimeBaseStructure.TIM_Prescaler	= 72-1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision	= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter	= 0; 
	TIM_TimeBaseInit(TIMEXPANDPWM,&TIM_TimeBaseStructure);
	
	//PWM1����
	TIM_OCInitStructure.TIM_OCMode 		=	TIM_OCMode_PWM1;//������ֵС���趨ֵʱ����͵�ƽ��������ֵԽ��ת��Խ��
	TIM_OCInitStructure.TIM_OutputState	=	TIM_OutputState_Disable;//����Ƚ�ʹ��
	TIM_OCInitStructure.TIM_OutputNState=	TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse		= 	0;//װ��Ƚ�����ֵ
	TIM_OCInitStructure.TIM_OCPolarity	=	TIM_OCPolarity_Low;//��ʼ�͵�ƽ
	TIM_OCInitStructure.TIM_OCNPolarity	=	TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState	= 	TIM_OCIdleState_Reset;//���е�ʱ������͵�ƽ��TIM1��Ч
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

//ģ�������ʼ��
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
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 //ADC��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;//�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 8;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�̶�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		//ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
 // ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 //ɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 8;	 	//Ҫת����ͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*����ADCʱ�ӣ�ΪPCLK2��6��Ƶ����12MHz,ADCƵ����߲��ܳ���14MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	/*����ADC1��ͨ��*/ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_55Cycles5);

	
	ADC_DMACmd(ADC1, ENABLE);  //ʹ��DMA
	ADC_Cmd(ADC1, ENABLE);	 //ʹ��ADC1  
	ADC_ResetCalibration(ADC1);  //��λУ׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADC1)); 	//�ȴ�У׼�Ĵ�����λ���
	ADC_StartCalibration(ADC1);  	//ADCУ׼
	while(ADC_GetCalibrationStatus(ADC1));  //�ȴ�У׼���
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//����û�в����ⲿ����������ʹ���������ADCת��

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
	


