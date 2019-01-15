#include "GlobalParam.h"


void CCDInit(void)
{
	GPIO_InitTypeDef 				GPIO_InitStructure; 
	ADC_InitTypeDef					ADC_InitStructure;
	/**************����CCD��SI��CLK�ܽ�************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=TSL_SI_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(TSL_SI_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=TSL_CLK_Pin;
	GPIO_Init(TSL_CLK_PORT,&GPIO_InitStructure);	
	/**************************************************/
	                 
	/***************����CCD��AO�ܽţ�����Ҫ����ADC*****************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC2|RCC_APB2Periph_AFIO,ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin		=	CCD_GPIO_PIN;   //ADC
	GPIO_InitStructure.GPIO_Mode	=	GPIO_Mode_AIN;	//ģ������
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

	ADC_ResetCalibration(CCD_ADC);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(CCD_ADC));//��ȡADC����У׼�Ĵ�����״̬
	
	ADC_StartCalibration(CCD_ADC);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(CCD_ADC));//��ȡָ��ADC��У׼����

	ADC_SoftwareStartConvCmd(CCD_ADC, DISABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������
	/*************************************************/
}

/****************��ȡCCD��ֵ******************************/
uint16_t Get_CCD_PixelValue( void )
{
	ADC_SoftwareStartConvCmd(CCD_ADC, ENABLE);
	while(!ADC_GetFlagStatus(CCD_ADC,ADC_FLAG_EOC));//ת��������־λ
	return ADC_GetConversionValue(CCD_ADC);	
}


/****************************
*����ʱ����������ʱ
*
***************************/

void delay_CCD()//��ʱi��ʱ������
{
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
}
/****************************
*�ع⺯����������CCD�������ʱ��
***************************/
void Integration(void)
{
	u8 i=0;
	TSL_CLK_H;
	TSL_SI_L; 
	delay_CCD();/*������ʱ100ns*/
		
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
		delay_CCD(); //�����ع�ʱ��
	} 
	TSL_CLK_L; 
	delay_CCD();
	TSL_CLK_H;
}

/****************************
*��ȡCCD��������
********************************/
void RD_TSL(uint16_t *ADV) 
{
	uint8_t i=0,tslp=0;
	TSL_CLK_H;
	TSL_SI_L; 
	delay_CCD();/*������ʱ100ns*/
 
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
		delay_CCD(); //�����ع�ʱ��
	} 
}



