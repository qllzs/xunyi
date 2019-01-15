#include "board.h"

uint8_t adc_value[8];        //传输数组
uint16_t adc_conv_value[4];  //消除偏差数组

void Stick_Init(void)
{
	int i = 0;
	/*消除硬件偏差*/
	adc_conv_value[0] = (2048*ADC_ConvertedValue[0])/2064;    
	adc_conv_value[1] = (2048*ADC_ConvertedValue[1])/1932;
	adc_conv_value[2] = (2048*ADC_ConvertedValue[2])/2064;
	adc_conv_value[3] = (2048*ADC_ConvertedValue[3])/1982;
	
	for(i=0;i<4;i++)
	{
		if(adc_conv_value[i] < 10)
		{
			adc_conv_value[i] = 0;
		}
		else if(adc_conv_value[i] > 4090)
		{
		  adc_conv_value[i] = 4095;
		}
	}
	
	/*赋值给传输数组*/
  adc_value[0]=((adc_conv_value[0]>>8)&0x00FF);//高八位   左边  左右
	adc_value[1]=((adc_conv_value[0]>>0)&0x00FF);//低八位
					
	adc_value[2]=((adc_conv_value[1]>>8)&0x00FF);//高八位  左边  上下
	adc_value[3]=((adc_conv_value[1]>>0)&0x00FF);//低八位
					  
	adc_value[4]=((adc_conv_value[2]>>8)&0x00FF);//高八位  右边  左右
	adc_value[5]=((adc_conv_value[2]>>0)&0x00FF);//低八位
					  
	adc_value[6]=((adc_conv_value[3]>>8)&0x00FF);//高八位  右边  上下
	adc_value[7]=((adc_conv_value[3]>>0)&0x00FF);//低八位

}



