#include "bsp_battery_adc.h"
uint16_t BatteryVoltage = 0,BatteryRemainder =0;
/**
	*@brief:ADC初始化函数
	*@param：无
	*@retval：无
	*/
void ReadBatteryVoltageADC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_APB2PeriphClockCmd(ADC_GPIO_CLK|RCC_APB2Periph_AFIO|RCC_APBxPeriph_ADCy,ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）

	GPIO_InitStructure.GPIO_Pin=ADC_PIN;  //GPIO_Pin_1; //ADC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(ADC_GPIO_PORT,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(ADC1, &ADC_InitStructure);
	
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADCx,ADC_Channel_x,1,ADC_SampleTime_239Cycles5);
	
	ADC_Cmd(ADCx,ENABLE);	

	ADC_ResetCalibration(ADCx);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(ADCx));//获取ADC重置校准寄存器的状态
	
	ADC_StartCalibration(ADCx);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(ADCx));//获取指定ADC的校准程序

	ADC_SoftwareStartConvCmd(ADCx, ENABLE);//使能或者失能指定的ADC的软件转换启动功能
	ReadBatteryVoltage(&FaultReg,&BatteryRemainder);//读取一次电池电压，初始化寄存器
}
/**
	*@brief:电池电量采集任务
	*@param：无
	*@retval：VolOfBattery--电池电压  单位：0.01V
	*/
uint8_t WarningCnt = 0,NormalCnt=0;  //警告次数，正常次数
uint16_t ReadBatteryVoltage( volatile uint16_t *FaultFlag,volatile uint16_t *remaider)
{
	uint32_t ConvertedValue= 0;
	uint16_t VolOfBattery = 0;
	uint8_t i = 0;
	//设置转换序列	  		 
//	ADCx->SQR3 &= 0XFFFFFFE0;//规则序列1 通道ch
//	ADCx->SQR3 |= CCD_ADC_CHANAL;
	for(i = 0;i < 50; i++)
	{
		ADC_SoftwareStartConvCmd(ADCx, ENABLE);
		while(!ADC_GetFlagStatus(ADCx,ADC_FLAG_EOC));//转换结束标志位
		ConvertedValue += ADC_GetConversionValue(ADCx);		
	}
	ConvertedValue = ConvertedValue/50;
//	VolOfBattery = (uint16_t)(((float)12*ConvertedValue*3.3/4096.0)*100);
//	if(VolOfBattery < (WARNING_VOLTAGE - WINDOW_VALUE))
//		*FaultFlag |= BATTERY_BIT;//电池电压过低则置相应标志位
//	if(VolOfBattery > (WARNING_VOLTAGE + WINDOW_VALUE))
//		*FaultFlag &= ~BATTERY_BIT;//电池电压正常则清除标志位
	
	VolOfBattery = (uint16_t)(((float)12*ConvertedValue*3.3/4096.0)*100);
	
	*remaider = (VolOfBattery-BASE_VOLTAGE)/3;
	
	if(VolOfBattery < WARNING_VOLTAGE)
	{
		if(NormalCnt > 0)
			NormalCnt = 0;
		WarningCnt++;
		if(WarningCnt > SoftFilterCnt)
		{
				*FaultFlag |= BATTERY_BIT;//电池电压过低则置相应标志位
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
			*FaultFlag &= ~BATTERY_BIT;//电池电压正常则清除标志位
			NormalCnt = 0;
		}
		
	}
	return VolOfBattery;
}







