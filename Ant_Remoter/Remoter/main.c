#include "board.h"

extern uint8_t adc_value[8];
int channel = 40;  //默认40通道传输

int main(void)
{
  u8 mode = 0;	//留待扩展其他功能模式切换
	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断优先级组别设置
	LED_Init();//初始化LED
	EXTIX_Init();//初始化按键中断
	SPI_Init();//初始化NRF所用SPI
	ADC1_Init();	//初始化ADC采样
  while(NRF_Check())//等待NRF连接是否正常
	{
		delay_ms(200);
	}
	
 	if(mode==0)          //发送模式
	{
		NRF_TX_MODE(&channel);//初始化为发送模式
		while(1)
		{
			//传输摇杆电位器值
			if(NRF_TxPacket(adc_value)==TX_OK)
				{		
					Stick_Init(); 					
				}
	  	LED_TX_BULING();//发送模式指示灯
		}
		
	}
}
