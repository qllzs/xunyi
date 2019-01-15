#ifndef __BSP_LED_H__
#define __BSP_LED_H__
#include "bsp_include.h"
//�����϶�������ɫLED0
#define LED0_GPIO_PORT    	GPIOB			              /* GPIO�˿� */
#define LED0_GPIO_CLK 	    RCC_APB2Periph_GPIOB		/* GPIO�˿�ʱ�� */
#define LED0_GPIO_PIN		    GPIO_Pin_3			        /* ���ӵ�SCLʱ���ߵ�GPIO */

// R-��ɫ
#define LED1_GPIO_PORT    	GPIOE			              /* GPIO�˿� */
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOE		/* GPIO�˿�ʱ�� */
#define LED1_GPIO_PIN		    GPIO_Pin_12			        /* ���ӵ�SCLʱ���ߵ�GPIO */

// G-��ɫ
#define LED2_GPIO_PORT    	GPIOE			              /* GPIO�˿� */
#define LED2_GPIO_CLK 	    RCC_APB2Periph_GPIOE		/* GPIO�˿�ʱ�� */
#define LED2_GPIO_PIN		    GPIO_Pin_10			        /* ���ӵ�SCLʱ���ߵ�GPIO */

// B-��ɫ
#define LED3_GPIO_PORT    	GPIOE			              /* GPIO�˿� */
#define LED3_GPIO_CLK 	    RCC_APB2Periph_GPIOE		/* GPIO�˿�ʱ�� */
#define LED3_GPIO_PIN		    GPIO_Pin_11			        /* ���ӵ�SCLʱ���ߵ�GPIO */


#define LED_GPIO_PORT       GPIOE
#define ON  0
#define OFF 1
#define LEDPIN         0x1C00   
/* ʹ�ñ�׼�Ĺ̼������IO*/
#define LED0(a)	if (a)	\
					GPIO_SetBits(LED0_GPIO_PORT,LED1_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED0_GPIO_PORT,LED1_GPIO_PIN)
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_GPIO_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_GPIO_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_GPIO_PIN)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 //����͵�ƽ
#define digitalToggle(p,i) {p->ODR ^=i;} //�����ת״̬


/* �������IO�ĺ� */
#define LED0_TOGGLE		 digitalToggle(LED0_GPIO_PORT,LED0_GPIO_PIN)
#define LED0_OFF		   digitalHi(LED0_GPIO_PORT,LED0_GPIO_PIN)
#define LED0_ON			   digitalLo(LED0_GPIO_PORT,LED0_GPIO_PIN)


#define LED1_TOGGLE		 digitalToggle(LED1_GPIO_PORT,LED1_GPIO_PIN)
#define LED1_OFF		   digitalHi(LED1_GPIO_PORT,LED1_GPIO_PIN)
#define LED1_ON			   digitalLo(LED1_GPIO_PORT,LED1_GPIO_PIN)

#define LED2_TOGGLE		 digitalToggle(LED2_GPIO_PORT,LED2_GPIO_PIN)
#define LED2_OFF		   digitalHi(LED2_GPIO_PORT,LED2_GPIO_PIN)
#define LED2_ON			   digitalLo(LED2_GPIO_PORT,LED2_GPIO_PIN)

#define LED3_TOGGLE		 digitalToggle(LED3_GPIO_PORT,LED3_GPIO_PIN)
#define LED3_OFF		   digitalHi(LED3_GPIO_PORT,LED3_GPIO_PIN)
#define LED3_ON			   digitalLo(LED3_GPIO_PORT,LED3_GPIO_PIN)

/* ������ɫ������߼��÷�ʹ��PWM�ɻ��ȫ����ɫ,��Ч������ */

//��
#define LED_RED  \
					LED1_ON;\
					LED2_OFF\
					LED3_OFF
#define LED_RED_PIN  (LED1_GPIO_PIN)
//��
#define LED_GREEN		\
					LED1_OFF;\
					LED2_ON\
					LED3_OFF
#define LED_GREEN_PIN  (LED2_GPIO_PIN)
//��
#define LED_BLUE	\
					LED1_OFF;\
					LED2_OFF\
					LED3_ON
#define LED_BLUE_PIN  (LED3_GPIO_PIN)
					
//��(��+��)					
#define LED_YELLOW	\
					LED1_ON;\
					LED2_ON\
					LED3_OFF
#define LED_YELLOW_PIN  (LED1_GPIO_PIN | LED2_GPIO_PIN)				
//��(��+��)
#define LED_PURPLE	\
					LED1_ON;\
					LED2_OFF\
					LED3_ON
#define LED_PURPLE_PIN  (LED1_GPIO_PIN | LED3_GPIO_PIN)				
					

//��(��+��)
#define LED_CYAN \
					LED1_OFF;\
					LED2_ON\
					LED3_ON
#define LED_CYAN_PIN  (LED2_GPIO_PIN | LED3_GPIO_PIN)						
					
//��(��+��+��)
#define LED_WHITE	\
					LED1_ON;\
					LED2_ON\
					LED3_ON
#define LED_WHITE_PIN  (LED1_GPIO_PIN | LED2_GPIO_PIN | LED3_GPIO_PIN)						
					
//��(ȫ���ر�)
#define LED_RGBOFF	\
					LED1_OFF;\
					LED2_OFF\
					LED3_OFF
					

void LED_GPIO_Init(void);
void led_blink(uint16_t led,uint16_t Blink_Time, FunctionalState NewState);









#endif  //__BSP_LED_H__



