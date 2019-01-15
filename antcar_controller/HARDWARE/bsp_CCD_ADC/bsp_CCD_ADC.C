#include "bsp_CCD_ADC.h"
u16 PixelValue_CCD[128]={0};

/**
	*@brief:ADC��ʼ������
	*@param����
	*@retval����
	*/
static void CCD_ADC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
//	
	RCC_APB2PeriphClockCmd(CCD_ADC_GPIO_CLK | RCC_APB2Periph_AFIO|CCD_RCC_APBxPeriph_ADCy,ENABLE);
	RCC_APB2PeriphClockCmd(CCD_ADC_GPIO_CLK , ENABLE);

	//	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��

	GPIO_InitStructure.GPIO_Pin=CCD_GPIO_PIN;  //GPIO_Pin_1; //ADC
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	
	GPIO_Init(CCD_GPIO_PORT,&GPIO_InitStructure);


	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfChannel = 1; 
	ADC_Init(CCD_ADCx, &ADC_InitStructure);
	
	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	ADC_RegularChannelConfig(CCD_ADCx,CCD_ADC_CHANAL,1,ADC_SampleTime_7Cycles5);
	
	ADC_Cmd(CCD_ADCx,ENABLE);	

	ADC_ResetCalibration(CCD_ADCx);//����ָ����ADC��У׼�Ĵ���
	while(ADC_GetResetCalibrationStatus(CCD_ADCx));//��ȡADC����У׼�Ĵ�����״̬
	
	ADC_StartCalibration(CCD_ADCx);//��ʼָ��ADC��У׼״̬
	while(ADC_GetCalibrationStatus(CCD_ADCx));//��ȡָ��ADC��У׼����

	ADC_SoftwareStartConvCmd(CCD_ADCx, DISABLE);//ʹ�ܻ���ʧ��ָ����ADC������ת����������
}
/**************************************************************************
�������ܣ�����CCD��ʼ��
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
**************************************************************************/
void  CCD_Init(void)
{ 
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//CCDʹ�õ���SI���š�CLK���ų�ʼ��
	RCC_APB2PeriphClockCmd(CCD_RCC_APBxPeriph_GPIOy,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=TSL_SI_Pin;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;	//ģ������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(TSL_SI_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=TSL_CLK_Pin;
	GPIO_Init(TSL_CLK_PORT,&GPIO_InitStructure);
	//��ʼ��adc
	CCD_ADC_Init();
}
/**
	*@brief:��ȡ����CCD�ĵ�������ADC�����ѹֵ
	*@param����
	*@retval��
	*/
static uint16_t Get_CCD_PixelValue( void )
{
//	//����ת������	  		 
//	CCD_ADCx->SQR3 &= 0XFFFFFFE0;//��������1 ͨ��ch
//	CCD_ADCx->SQR3 |= CCD_ADC_CHANAL;
	ADC_SoftwareStartConvCmd(CCD_ADCx, ENABLE);
	while(!ADC_GetFlagStatus(CCD_ADCx,ADC_FLAG_EOC));//ת��������־λ
	return ADC_GetConversionValue(CCD_ADCx);		
}
/*
@notice :�𷢵�̫�죬�����˵�
*/
void debug_ccd(uint16_t * ADV)
{
	uint8_t i=0;
	USART_SendData(USART3, 0x54);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	for(i=0;i<128;i++)
	{
		USART_SendData(USART3, (uint8_t)(ADV[i]>>8));
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
		USART_SendData(USART3, (uint8_t) ADV[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);	
	}
//	for(i=0;i<128;i++)
//	{
//		printf("%4d ",ADV[i]);
//	}
//	printf("\n\r");
}
void delay_CCD()//��ʱi��ʱ������
{
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	
//	__nop();
//	__nop();
//	__nop();
//	__nop();
//	__nop();
//	
//	__nop();
//	__nop();
//	__nop();
//	__nop();
//	__nop();
}
/**************************************************************************
*@brief���ع⺯��
*@param����
*@retval����
**************************************************************************/
void Integration(void)
{
	u8 i=0;
  TSL_CLK_H;
  TSL_SI_L; 
  delay_CCD();/*������ʱ100ns*/
      
   TSL_SI_H;
  TSL_CLK_L;
 
  //delay_us(10);
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
	//delay_us(200);
}


/**************************************************************************
*@brief��CCD���ݲɼ�
*@param��ADV-��������ָ��
*@retval����
**************************************************************************/
void RD_TSL(uint16_t *ADV) 
{
  u8 i=0,tslp=0;
  TSL_CLK_H;
  TSL_SI_L; 
  delay_CCD();/*������ʱ100ns*/
  

	TSL_SI_H; 	
  TSL_CLK_L;
	
 // delay_us(1);
  delay_CCD();    
  
  
	TSL_CLK_H;
	TSL_SI_L;
  delay_CCD(); 
  for(i=0;i<128;i++)
  { 
    TSL_CLK_L; 
  //�����ع�ʱ��

		delay_CCD();


    ADV[tslp] = Get_CCD_PixelValue();
    ++tslp;
    TSL_CLK_H;
    delay_CCD(); //�����ع�ʱ��
		//delay_us(1);
  } 
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


	
/**************************************************************************
*@brief���������CCD�õ�����ֵ���߿�
*@param����ֵ���߿�
*@param��PixelValue -ָ��洢CCD��ȡ�������ݵ������ָ��
*@param��Car_Sta -ָ��洢CCD��ȡ�������ݵ������ָ��
*@retval����ֵ
**************************************************************************/
Str_Line BlackLine[5];
uint8_t	LineCnt=0,ForGiveTime=0,ForGiveCCDTime=0,ForGiveHaveLineTime=0,ForGiveTwo2OneTime=0;
u16 PixelValue_Max = 0,PixelValue_Min = 0,CCD_Threshold = 0;
uint8_t LineCenterOld = 0,LineWidthOld = 0,LineGapOld=0;
int16_t  BeginLeft=0,EndRight=0;
uint8_t FindLineStatu=0;
//void  Find_CCD_Center_Width(uint8_t *LineCenter,uint8_t *LineWidth,uint16_t *PixelValue_CCD,volatile uint8_t *Line_Sta)
void Find_CCD_Center_Width(void* addr[6])
{	 
	uint8_t *pLineCenter,*pLineWidth,*pLineCnt,LineCnt,*pLineGap;
	uint16_t *PixelValue_CCD;
	volatile uint8_t *Line_Sta;
	u16 PixelValue[128] = {0};
	uint8_t i = 0,j=0;
	
	pLineCenter 		= addr[0];
	pLineWidth  		= addr[1];
	PixelValue_CCD	= addr[2];
	Line_Sta				= addr[3];
	pLineCnt				= addr[4];
	pLineGap				= addr[5];
	
	
	*pLineGap = 0;
//		uint16_t diffvalue;
	memcpy(PixelValue,PixelValue_CCD,128*2);
	memset(&BlackLine,0,sizeof(Str_Line)*5);
	LineCnt =0;
	
	PixelValue_Max=PixelValue[5];  //��̬��ֵ�㷨����ȡ������Сֵ

	for(i=5;i<123;i++)   //���߸�ȥ��5����
	{
		if(PixelValue_Max<=PixelValue[i])
			PixelValue_Max=PixelValue[i];
	 }
	
	PixelValue_Min=PixelValue[5];  //��Сֵ
	for(i=5;i<123;i++) 
	{
		if(PixelValue_Min>=PixelValue[i])
			PixelValue_Min=PixelValue[i];
	}
		
		 	 
	CCD_Threshold=(PixelValue_Max+PixelValue_Min)/3;	  //���������������ȡ����ֵ������ֵ������ɫ
	for(i=5;i<118;i++)
	{
		if((PixelValue[i] < CCD_Threshold) && (PixelValue[i+1] > CCD_Threshold) && (PixelValue[i+2] < CCD_Threshold))
		{
			PixelValue[i+1] = PixelValue[i];
			i = i+1;
		}
		else if((PixelValue[i] > CCD_Threshold) && (PixelValue[i+1] < CCD_Threshold) && (PixelValue[i+2] > CCD_Threshold))
		{
			PixelValue[i+1] = PixelValue[i];
			i = i+1;
		}
		else if((PixelValue[i] < CCD_Threshold) && (PixelValue[i+1] < CCD_Threshold) && (PixelValue[i+2] > CCD_Threshold)&& (PixelValue[i+3] > CCD_Threshold)&&(PixelValue[i+4] < CCD_Threshold)&&(PixelValue[i+5] < CCD_Threshold))
		{
			PixelValue[i+2] = PixelValue[i+3]= PixelValue[i];
			i = i+4;
		}
		else if((PixelValue[i] > CCD_Threshold) && (PixelValue[i+1] > CCD_Threshold) && (PixelValue[i+2] < CCD_Threshold)&& (PixelValue[i+3] < CCD_Threshold)&&(PixelValue[i+4] > CCD_Threshold)&&(PixelValue[i+5] > CCD_Threshold))
		{
			PixelValue[i+2] = PixelValue[i+3]= PixelValue[i];
			i = i+4;
		}
	}
	for(i=5;i<118;i++)
	{
		if((PixelValue[i] < CCD_Threshold) && (PixelValue[i+1] > CCD_Threshold) && (PixelValue[i+2] < CCD_Threshold))
		{
			PixelValue[i+1] = PixelValue[i];
			i = i+1;
		}
		else if((PixelValue[i] > CCD_Threshold) && (PixelValue[i+1] < CCD_Threshold) && (PixelValue[i+2] > CCD_Threshold))
		{
			PixelValue[i+1] = PixelValue[i];
			i = i+1;
		}
	}
	//	if(LineCenterOld-3*LineWidthOld*2)
	 //���ϴ����ĵ�������
	if(( LineCenterOld !=0) && (LineWidthOld != 0))
	{
		BeginLeft = (LineCenterOld > (3*LineWidthOld/2+5))? (LineCenterOld - 3*LineWidthOld/2): 5;
		EndRight  = (LineCenterOld < (118 - 3*LineWidthOld/2))? (LineCenterOld + 3*LineWidthOld/2): 118;
	}
	else
	{
		BeginLeft = 5;
		EndRight = 118;
	}
	if(ModeReg == 0)
	{
		BeginLeft = 5;
		EndRight = 118;
	}
	for(i = BeginLeft;i<EndRight; i++)   
	{
		if(LineCnt == 5)
			break;
		if(PixelValue[i]>CCD_Threshold&&PixelValue[i+1]>CCD_Threshold&&PixelValue[i+2]>CCD_Threshold&&PixelValue[i+3]<CCD_Threshold&&PixelValue[i+4]<CCD_Threshold&&PixelValue[i+5]<CCD_Threshold)
		{	
			BlackLine[LineCnt].left = i+2; 
			for(j=i+5;j < EndRight; j++)
			{
				if(PixelValue[j]<CCD_Threshold&&PixelValue[j+1]<CCD_Threshold&&PixelValue[j+2]<CCD_Threshold&&PixelValue[j+3]>=CCD_Threshold&&PixelValue[j+4]>=CCD_Threshold&&PixelValue[j+5]>=CCD_Threshold)
				{
					BlackLine[LineCnt].right = j+2;
					BlackLine[LineCnt].width = DiffTick(BlackLine[LineCnt].right,BlackLine[LineCnt].left);
					LineCnt++;
					i=j;
					break;
				}
				else if((j==118)&&(PixelValue[118]<CCD_Threshold&&PixelValue[119]<CCD_Threshold&&PixelValue[120]<CCD_Threshold&&PixelValue[121]<CCD_Threshold&&PixelValue[122]<CCD_Threshold&&PixelValue[123]<CCD_Threshold))
				{
					BlackLine[LineCnt].right = 123;
					BlackLine[LineCnt].width = DiffTick(BlackLine[LineCnt].right,BlackLine[LineCnt].left);
					LineCnt++;
					i=118;
					break;
				}
			}
		}
	}
		
	if( !SDOK )
	{
		#ifndef Debug_LineWidth
			if(UpDownChangeMode)
		#endif
		{
			memset(WriteBuffer,0,sizeof(WriteBuffer));
			sprintf(WriteBuffer,"\n %6d,%d,%3d,%3d,%3d,%3d,%3d,%3d",TimeStampReg,LineCnt,BlackLine[0].width,BlackLine[0].left,BlackLine[0].right,BlackLine[1].width,BlackLine[1].left,BlackLine[1].right);
			SDOK = fileWrite(WriteBuffer,"0:CCData.txt");
		}
	}
	
	
	//***********************�������ݴ���*****************//
	if(LineCnt == 1)  //�����⵽1��������ֱ����
	{
		if(*Line_Sta == 0)
		{
			ForGiveHaveLineTime++;
			if(ForGiveHaveLineTime < ThreFindHaveLineCnt)
			{
				*Line_Sta = 0;
				*pLineCnt = 0;
				FindLineStatu = 0;
				return;
			}					
		}
		ForGiveHaveLineTime = 0;
		ForGiveTime = 0;
		*pLineWidth = BlackLine[0].width;
		*pLineCenter = (BlackLine[0].left + BlackLine[0].right)/2;
		*pLineCnt = 1;
		if(((BlackLine[0].left > 95) && (BlackLine[0].width > 15)) || ((BlackLine[0].right < 30) && (BlackLine[0].width > 15)))
		{
			*Line_Sta = 0;
			*pLineWidth = 0;
			*pLineCenter = 0;
			*pLineCnt = 0;
			FindLineStatu = 0;
			*pLineCnt = 0;
			return;
		}
	
//			memset(WriteBuffer,0,sizeof(WriteBuffer));
//			sprintf(WriteBuffer,"\n�������ң�W=%2d,L=%2d,R=%d,Max=%d,Min=%4d,Thr=%4d",BlackLine[0].width,BlackLine[0].left,BlackLine[0].right,PixelValue_Max,PixelValue_Min,CCD_Threshold);
//			fileWrite(WriteBuffer,"0:��������.txt");
		
	#ifndef HollowStopLine
		if((DiffTick(LineWidthOld,BlackLine[0].width) >= ThreWidthJump) && (BlackLine[0].width < LINE_WIDTH_MAX))
		{	
			
			ForGiveCCDTime = 0;	
			*Line_Sta = 1; 
			LineCenterOld = *pLineCenter;
			LineWidthOld = *pLineWidth;
			ForGiveTime=0;
		}
		else if(((DiffTick(LineWidthOld,BlackLine[0].width) >= ThreWidthJump) && (LineWidthOld < LINE_WIDTH_MAX)))
		{
			ForGiveCCDTime++;
			if( ForGiveCCDTime > ThreWidthJumpCnt)
			{
				ForGiveCCDTime = 0;
				ForGiveTime=0;
				*Line_Sta = 0; 
				*pLineWidth = 0;
				*pLineCenter =0;
				*pLineCnt = 0;
				return;
			}
			*pLineCenter = LineCenterOld;
			*pLineWidth = LineWidthOld ;
			*Line_Sta = 1;
			return;
		}
		else
	#endif	
		{
			if(FindLineStatu == 2)
			{
				ForGiveTwo2OneTime++;
				if(ForGiveTwo2OneTime > ThreFindNOLineCnt)
				{
					*Line_Sta = 0;
					ForGiveTwo2OneTime = 0;
					FindLineStatu = 0;
					if(ModeReg == 1)
						ModeReg = 3;
					return;
				}
				*pLineGap = LineGapOld;
				*pLineCenter = LineCenterOld;
				*pLineCnt = 2;
				FindLineStatu = 2;
				*Line_Sta = 1;
			}
			else
			{
				ForGiveCCDTime = 0;	
				ForGiveTime=0;
				*Line_Sta = 1; 
				LineCenterOld = *pLineCenter;
				LineWidthOld = *pLineWidth ;
				FindLineStatu = 1;
			}
		}
	}
	else if(LineCnt == 0) //���û�м�⵽���ߣ����ʱΪ�ϲ����ģʽ
	{
		if(ModeReg == 1)
		{
			ForGiveTime++;
//			if(ForGiveTime == 18)
//			{
//				ForGiveTime = 18;
//			}
			if(ForGiveTime >= ThreFindNOLineCnt)
			{
				*Line_Sta = 0; 
				*pLineWidth = 0;
				*pLineCenter =0;
				//FindLineStatu = 0;
				ModeReg = 3;
				return;
			}

				*pLineCenter = LineCenterOld;
				*pLineWidth = LineWidthOld ;	
				*Line_Sta = 1; 
				*pLineCnt = FindLineStatu;
		}
//			memset(WriteBuffer,0,sizeof(WriteBuffer));
//			sprintf(WriteBuffer,"\n�������ң�%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",PixelValue_Max,PixelValue_Min,CCD_Threshold,BlackLine[0].left,BlackLine[0].right,
//						PixelValue[BlackLine[0].left],PixelValue[BlackLine[0].left+1],PixelValue[BlackLine[0].left+2],PixelValue[BlackLine[0].left+3],
//			PixelValue[BlackLine[0].left+4],PixelValue[BlackLine[0].left+5],PixelValue[BlackLine[0].left+6],PixelValue[BlackLine[0].left+7],PixelValue[BlackLine[0].left+8],
//			PixelValue[BlackLine[0].left+9],PixelValue[BlackLine[0].left+10],PixelValue[BlackLine[0].left+11],PixelValue[BlackLine[0].left+12]);
//			fileWrite(WriteBuffer,"0:�޺�������.txt");
	}
	else  //��⵽�������������Ҵ���Ѱ��ģʽ�Ľ���״̬�Ƚϣ������Ѱ�ߣ����쳣��δ��⵽��
	{
		ForGiveTime=0;
		if(ModeReg == 1)
		{
//				memset(WriteBuffer,0,sizeof(WriteBuffer));
//				sprintf(WriteBuffer,"\n�������ң�Cnt=%d,L0=%2d,R0=%d,L1=%2d,R1=%d,OW=%2d,OC=%d",LineCnt,BlackLine[0].left,BlackLine[0].right,BlackLine[1].left,BlackLine[1].right,LineWidthOld,LineCenterOld);
//				fileWrite(WriteBuffer,"0:����������.txt");
			#ifdef HollowStopLine
			if(LineCnt == 2)
			{
				if((BlackLine[0].width < LINE_WIDTH_MAX) && (BlackLine[1].width < LINE_WIDTH_MAX))
				{
					*pLineGap = DiffTick(BlackLine[1].left,BlackLine[0].right);
					*pLineCenter = (BlackLine[1].left+BlackLine[0].right)/2;
					FindLineStatu = 2;
					*pLineCnt = 2;
					ForGiveTwo2OneTime = 0;
					LineGapOld = *pLineGap;
					LineCenterOld = *pLineCenter;
				}
				else
				{
					FindLineStatu = 3;
					*pLineGap  = 0; 
					ModeReg = 3;
					*pLineCnt = 0;
				}
			}
		
			#else
				for(i = 0; i < LineCnt;i++)
				{
					if((DiffTick(BlackLine[i].width,LineWidthOld) < ThreLineDiff) && (DiffTick((BlackLine[i].left + BlackLine[i].right)/2,LineCenterOld) < ThrePostDiff))
					{
						*pLineWidth = BlackLine[i].width;
						*pLineCenter = (BlackLine[i].left + BlackLine[i].right)/2;
						LineCenterOld = *pLineCenter;
						LineWidthOld = *pLineWidth;
						*Line_Sta = 1; 
						return;
					}
				}
				*Line_Sta = 0; 
				*pLineWidth = 0;
				*pLineCenter =0;
				ModeReg = 3;
				return;
			#endif
		}
		else
		{
			*pLineCnt = LineCnt;
		}
	}
	
}
/**************************************************************************
�������ܣ�ת�����  Ѳ��
��ڲ�����CCD��ȡ������
����  ֵ�����ٶ�ֵang
**************************************************************************/
float turn(u8 CCD_zhongzhi)  //ת�����
{
	  float ang;     
    float Bias;	  
	  Bias=CCD_zhongzhi-64;
	  ang=Bias*K;
	  return ang;
}
long CCD_AVG_E,CCD_AVG = 0,PreCCD_AVG=0;
float	I=0;
int8_t	ChangeI;
uint8_t UpCnt=0,DownCnt=0;
void CalculateIntegrationTime(uint16_t *CCD_AD,int8_t *IntTime)
{
    unsigned char i;
    long CCD_SUM=0;
    // �������ص��ƽ��ADֵ 
    for(i=PIX_START;i<PIX_END;i++)
    {
    	CCD_SUM += CCD_AD[i];
    }
    CCD_AVG = (CCD_SUM / PIX_NUM);   //��ƽ��ֵ
		
		/*
		I = Kp*(CCD_AVG - BestAVG) + Kd*(CCD_AVG - PreCCD_AVG);
		
		
		
		if( I >= 0.0)
		{
			ChangeI = (uint8_t)I;
			if(ChangeI >= (*IntTime) )
				*IntTime = 0;
			else
				*IntTime = *IntTime - ChangeI;

		}
		else
		{
			ChangeI = (uint8_t)(-I);
			
			if(((*IntTime) + ChangeI)  < INT_TIME_MAX)
				*IntTime = *IntTime + ChangeI;
			else
				*IntTime = INT_TIME_MAX;
						
		}
		*/
		
    CCD_AVG_E = (int)2500 - (int)CCD_AVG;
    
    if(CCD_AVG_E < -300)
		{
			if(UpCnt == 0)
			{
				if((++DownCnt) == CCDDownTime)
				{
					*IntTime = *IntTime - 2;
					DownCnt=0;
				}			
			}
			else
			{
				UpCnt = 0;
				DownCnt = 0;
			}
        
		}
    if(CCD_AVG_E > 200)
		{
			if(DownCnt == 0)
			{
				if((++UpCnt) == CCDUpTime)
				{
					*IntTime = *IntTime + 2;
					UpCnt = 0;
				}
			}
			else
			{
				UpCnt = 0;
				DownCnt = 0;
			}
			
		}
    if(*IntTime < 1)
        *IntTime = 0;
    if(*IntTime >= INT_TIME_MAX)
        *IntTime = INT_TIME_MAX;

}















