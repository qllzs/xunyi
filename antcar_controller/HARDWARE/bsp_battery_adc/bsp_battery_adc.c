#include "bsp_battery_adc.h"
uint16_t BatteryVoltage = 0,BatteryRemainder =0;
/**
	*@brief:ADC��ʼ������
	*@param����
	*@retval����
	*/
void ReadBatteryVoltageADC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(ADC_GPIO_CLK|RCC_APB2Periph_AFIO|RCC_APBxPeriph_ADCy,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

	GPIO_InitStructure.GPIO_Pin=ADC_PIN;  //GPIO_Pin_1; //ADC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(ADC_GPIO_PORT,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(ADCx,ADC_Channel_x,1,ADC_SampleTime_239Cycles5);
	
	ADC_Cmd(ADCx,ENABLE);	

	ADC_ResetCalibration(ADCx);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(ADCx));//��ȡADC����У׼�Ĵ�����״̬
	
	ADC_StartCalibration(ADCx);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(ADCx));//��ȡָ��ADC��У׼����

	ADC_SoftwareStartConvCmd(ADCx, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
	ReadBatteryVoltage(&FaultReg,&BatteryRemainder);//��ȡһ�ε�ص�ѹ����ʼ���Ĵ���
}
/**
	*@brief:��ص����ɼ�����
	*@param����
	*@retval��VolOfBattery--��ص�ѹ  ��λ��0.01V
	*/
uint8_t WarningCnt = 0,NormalCnt=0;  //�����������������
uint16_t ReadBatteryVoltage( volatile uint16_t *FaultFlag,volatile uint16_t *remaider)
{
	uint32_t ConvertedValue= 0;
	uint16_t VolOfBattery = 0;
	uint8_t i = 0;
	//����ת������	  		 
//	ADCx->SQR3 &= 0XFFFFFFE0;//��������1 ͨ��ch
//	ADCx->SQR3 |= CCD_ADC_CHANAL;
	for(i = 0;i < 50; i++)
	{
		ADC_SoftwareStartConvCmd(ADCx, ENABLE);
		while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_EOC));//ת��������־λ
		ConvertedValue += ADC_GetConversionValue(ADCx);		
	}
	ConvertedValue = ConvertedValue/50;
//	VolOfBattery = (uint16_t)(((float)12*ConvertedValue*3.3/4096.0)*100);
//	if(VolOfBattery < (WARNING_VOLTAGE - WINDOW_VALUE))
//		*FaultFlag |= BATTERY_BIT;//��ص�ѹ����������Ӧ��־λ
//	if(VolOfBattery > (WARNING_VOLTAGE + WINDOW_VALUE))
//		*FaultFlag &= ~BATTERY_BIT;//��ص�ѹ�����������־λ
	
	VolOfBattery = (uint16_t)(((float)12*ConvertedValue*3.3/4096.0)*100);
	
	*remaider = (VolOfBattery-BASE_VOLTAGE)/3;
	
	if(VolOfBattery < WARNING_VOLTAGE)
	{
		if(NormalCnt > 0)
			NormalCnt = 0;
		WarningCnt++;
		if(WarningCnt > SoftFilterCnt)
		{
				*FaultFlag |= BATTERY_BIT;//��ص�ѹ����������Ӧ��־λ
				WarningCnt = 0;
		}
	
	}
	else
	{
		if(WarningCnt != 0)
			WarningCnt = 0;
		NormalCnt++;
		if(NormalCnt > SoftFilterCnt)
		{
			*FaultFlag &= ~BATTERY_BIT;//��ص�ѹ�����������־λ
			NormalCnt = 0;
		}
		
	}
	return VolOfBattery;
}







