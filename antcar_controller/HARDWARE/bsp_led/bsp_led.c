#include "bsp_led.h"
/**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Init(void)
{		
		/*定义一个GPIO_InitTypeDef类型的结构体*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*开启LED相关的GPIO外设时钟*/
		RCC_APB2PeriphClockCmd( LED0_GPIO_CLK | LED1_GPIO_CLK | LED2_GPIO_CLK | LED3_GPIO_CLK, ENABLE);
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = LED0_GPIO_PIN;	

		/*设置引脚模式为通用推挽输出*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*设置引脚速率为50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*调用库函数，初始化GPIO*/
		GPIO_Init(LED0_GPIO_PORT, &GPIO_InitStructure);	
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;

		/*调用库函数，初始化GPIO*/
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);		
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;

		/*调用库函数，初始化GPIO*/
		GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
		
		/*选择要控制的GPIO引脚*/
		GPIO_InitStructure.GPIO_Pin = LED3_GPIO_PIN;

		/*调用库函数，初始化GPIOF*/
		GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

		/* 关闭所有led灯	*/
		GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
		GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);	 
		GPIO_SetBits(LED3_GPIO_PORT, LED3_GPIO_PIN);
}
 /**
  * @brief  led灯各种颜色以某种频率闪烁作为指示
  * @param  led --参考.h文件中各种颜色的宏定义
  * 				Blink_Time-闪烁周期，单位ms
  * @retval 无
  */
void led_blink(uint16_t led,uint16_t Blink_Time, FunctionalState NewState)
{
	uint8_t i;

	switch(led)
	{
		case 0:
				for(i=0;i<Blink_Time;i++)
				{
					LED_GREEN;
					delay_ms(500);
					LED_RGBOFF;
					delay_ms(500);
				}
		break;
	}
}















