#ifndef __BUMPER_H
#define	__BUMPER_H
#include "GlobalParam.h"

#define	FrontBumperTrig	 			0x01
#define	BackBumperTrig				0x02
#define	BothBumperTrig				0x03

#define	UltrasonicLen				13
/****************BumperÅäÖÃ****************************/
#define BUMPER_A_GPIO_PORT         GPIOA
#define BUMPER_A_GPIO_CLK          (RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO)
#define BUMPER_A_GPIO_PIN          GPIO_Pin_3
#define BUMPER_A_EXTI_PORTSOURCE   GPIO_PortSourceGPIOA
#define BUMPER_A_EXTI_PINSOURCE    GPIO_PinSource3
#define BUMPER_A_EXTI_LINE         EXTI_Line3
#define BUMPER_A_EXTI_IRQ          EXTI3_IRQn
#define	BUMPER_AIRQ_Handler		   EXTI3_IRQHandler
#define BUMPER_AIRQ_READ()  	  ((BUMPER_A_GPIO_PORT->IDR & BUMPER_A_GPIO_PIN) != 0)

#define BUMPER_B_GPIO_PORT         GPIOA
#define BUMPER_B_GPIO_CLK          (RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO)
#define BUMPER_B_GPIO_PIN          GPIO_Pin_4
#define BUMPER_B_EXTI_PORTSOURCE   GPIO_PortSourceGPIOA
#define BUMPER_B_EXTI_PINSOURCE    GPIO_PinSource4
#define BUMPER_B_EXTI_LINE         EXTI_Line4
#define BUMPER_B_EXTI_IRQ          EXTI4_IRQn
#define	BUMPER_BIRQ_Handler		   EXTI4_IRQHandler
#define BUMPER_BIRQ_READ()  	  ((BUMPER_B_GPIO_PORT->IDR & BUMPER_B_GPIO_PIN) != 0)
/**************************************************************************/

/**********³¬Éù´®¿ÚÅäÖÃ**********************/
#define UltrasonicCLK				RCC_APB2Periph_USART1
#define UltrasonicGPIO_CLK			RCC_APB2Periph_GPIOA
#define	UltrasonicGPIO_PORT			GPIOA
#define	UltrasonicTX_PIN			GPIO_Pin_9
#define	UltrasonicRX_PIN			GPIO_Pin_10
#define	UltrasonicUart				USART1
#define UltrasonicIRQ          		USART1_IRQn
#define	UltrasonicHandler			USART1_IRQHandler

/*******µ÷ÊÔ´®¿ÚÅäÖÃ**************/
#define DebugUartCLK				RCC_APB1Periph_USART3
#define DebugUartGPIO_CLK			RCC_APB2Periph_GPIOB
#define	DebugUartGPIO_PORT			GPIOB
#define	DebugUartTX_PIN				GPIO_Pin_10
#define	DebugUartRX_PIN				GPIO_Pin_11
#define	DebugUart					USART3
#define DebugUartIRQ          		USART3_IRQn
#define	DebugUartHandler			USART3_IRQHandler


void UltrasonicUartInit(uint32_t bound1,uint32_t bound2);
void BumperTriggerInit(void);
void BumperInit(void);
#endif

