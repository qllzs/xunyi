#include "bsp_led.h"
/**
  * @brief  ��ʼ������LED��IO
  * @param  ��
  * @retval ��
  */
void LED_GPIO_Init(void)
{		
		/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
		GPIO_InitTypeDef GPIO_InitStructure;

		/*����LED��ص�GPIO����ʱ��*/
		RCC_APB2PeriphClockCmd( LED0_GPIO_CLK | LED1_GPIO_CLK | LED2_GPIO_CLK | LED3_GPIO_CLK, ENABLE);
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = LED0_GPIO_PIN;	

		/*��������ģʽΪͨ���������*/
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

		/*������������Ϊ50MHz */   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(LED0_GPIO_PORT, &GPIO_InitStructure);	
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);		
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;

		/*���ÿ⺯������ʼ��GPIO*/
		GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
		
		/*ѡ��Ҫ���Ƶ�GPIO����*/
		GPIO_InitStructure.GPIO_Pin = LED3_GPIO_PIN;

		/*���ÿ⺯������ʼ��GPIOF*/
		GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

		/* �ر�����led��	*/
		GPIO_SetBits(LED0_GPIO_PORT, LED0_GPIO_PIN);
		GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);	 
		GPIO_SetBits(LED3_GPIO_PORT, LED3_GPIO_PIN);
}
 /**
  * @brief  led�Ƹ�����ɫ��ĳ��Ƶ����˸��Ϊָʾ
  * @param  led --�ο�.h�ļ��и�����ɫ�ĺ궨��
  * 				Blink_Time-��˸���ڣ���λms
  * @retval ��
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















