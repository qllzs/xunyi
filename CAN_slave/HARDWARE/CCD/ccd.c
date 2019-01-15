#include "GlobalParam.h"


void CCDInit(void)
{
	GPIO_InitTypeDef 				GPIO_InitStructure; 
	ADC_InitTypeDef					ADC_InitStructure;
	/**************配置CCD的SI与CLK管脚************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=TSL_SI_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//模拟输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(TSL_SI_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=TSL_CLK_Pin;
	GPIO_Init(TSL_CLK_PORT,&GPIO_InitStructure);	
	/**************************************************/
	                 
	/***************配置CCD的AO管脚，其需要配置ADC*****************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC2|RCC_APB2Periph_AFIO,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin		=	CCD_GPIO_PIN;   //ADC
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AIN;	//模拟输入
	GPIO_InitStructure.GPIO_Speed	=	GPIO_Speed_50MHz;
	
	GPIO_Init(CCD_GPIO_PORT,&GPIO_InitStructure);
	
	ADC_InitStructure.ADC_Mode 								= ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode 				= DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode 	= DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv 		= ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign 					= ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel 				= 1; 
	ADC_Init(CCD_ADC, &ADC_InitStructure);
	
	ADC_RegularChannelConfig(CCD_ADC,CCD_ADC_CHANAL,1,ADC_SampleTime_7Cycles5);
	
	ADC_Cmd(CCD_ADC,ENABLE);	

	ADC_ResetCalibration(CCD_ADC);//重置指定的ADC的校准寄存器
	while(ADC_GetResetCalibrationStatus(CCD_ADC));//获取ADC重置校准寄存器的状态
	
	ADC_StartCalibration(CCD_ADC);//开始指定ADC的校准状态
	while(ADC_GetCalibrationStatus(CCD_ADC));//获取指定ADC的校准程序

	ADC_SoftwareStartConvCmd(CCD_ADC, DISABLE);//使能或者失能指定的ADC的软件转换启动功能
	/*************************************************/
}

/****************获取CCD的值******************************/
uint16_t Get_CCD_PixelValue( void )
{
	ADC_SoftwareStartConvCmd(CCD_ADC, ENABLE);
	while(!ADC_GetFlagStatus(CCD_ADC,ADC_FLAG_EOC));//转换结束标志位
	return ADC_GetConversionValue(CCD_ADC);	
}


/****************************
*根据时序来配置延时
*
***************************/

void delay_CCD()//延时i个时钟周期
{
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
}
/****************************
*曝光函数，来调节CCD晶体积分时间
***************************/
void Integration(void)
{
	u8 i=0;
	TSL_CLK_H;
	TSL_SI_L; 
	delay_CCD();/*合理延时100ns*/
		
	TSL_SI_H;
	TSL_CLK_L;
 
	delay_CCD();    
	TSL_CLK_H;
	TSL_SI_L;
	delay_CCD();
		
	for(i=0;i<128;i++)
	{ 
		TSL_CLK_L; 
		delay_CCD();
		TSL_CLK_H;
		delay_CCD(); //调节曝光时间
	} 
	TSL_CLK_L; 
	delay_CCD();
	TSL_CLK_H;
}

/****************************
*读取CCD像素数据
********************************/
void RD_TSL(uint16_t *ADV) 
{
	uint8_t i=0,tslp=0;
	TSL_CLK_H;
	TSL_SI_L; 
	delay_CCD();/*合理延时100ns*/
 
	TSL_SI_H; 	
	TSL_CLK_L;
	
	delay_CCD();    
  
	TSL_CLK_H;
	TSL_SI_L;
	delay_CCD(); 
	for(i=0;i<128;i++)
	{ 
		TSL_CLK_L; 
		delay_CCD();
		ADV[tslp] = Get_CCD_PixelValue();
		++tslp;
		TSL_CLK_H;
		delay_CCD(); //调节曝光时间
	} 
}



