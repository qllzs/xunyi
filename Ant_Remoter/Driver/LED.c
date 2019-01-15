#include "board.h"


void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);      /*使能SWD 禁用JTAG*/
	RCC_APB2PeriphClockCmd(RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = Pin_LED_0|Pin_LED_1;
	GPIO_Init(GPIO_LED, &GPIO_InitStructure);
}

void LED_0_ON(void)
{
	GPIO_SetBits(GPIO_LED, Pin_LED_0);	
}

void ANO_LED_0_OFF(void)
{
	GPIO_ResetBits(GPIO_LED, Pin_LED_0);			
}

void ANO_LED_1_ON(void)
{
	GPIO_SetBits(GPIO_LED, Pin_LED_1);	
}

void ANO_LED_1_OFF(void)
{
	GPIO_ResetBits(GPIO_LED, Pin_LED_1);			
}


void LED_TX_BULING(void)
{

	LED_0_ON();
	delay_ms(200);
	ANO_LED_0_OFF();
	delay_ms(200);
}


void LED_RX_BULING(void)
{
	LED_0_ON();
	delay_ms(100);
	ANO_LED_0_OFF();
	delay_ms(100);
	LED_0_ON();
	delay_ms(100);
	ANO_LED_0_OFF();
	delay_ms(100);

}


