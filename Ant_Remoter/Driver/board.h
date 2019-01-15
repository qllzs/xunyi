#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f10x.h"
#include "sysconfig.h"
#include "Nrf24l01.h"
#include "delay.h"
#include "sys.h"
#include "SPI1.h"
#include "ADC.h"
#include "LED.h"
#include "KEY.h"
#include "stick.h"
#include "Oled.h"
#include "filter.h"
#include "exti.h"
#include "stm32f10x_exti.h"

//#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */
/***************LED GPIO定义******************/
#define RCC_LED						RCC_APB2Periph_GPIOB
#define GPIO_LED					GPIOB
#define Pin_LED_0					GPIO_Pin_15
#define Pin_LED_1					GPIO_Pin_13
/*********************************************/
/***************KEY GPIO定义******************/
#define RCC_KEY						RCC_APB2Periph_GPIOB
#define GPIO_KEY					GPIOB
#define Pin_KEY0					GPIO_Pin_12
#define Pin_KEY1					GPIO_Pin_14
/*********************************************/

/***************SPI GPIO定义******************/
#define RCC_SPI_GPIO_SPI		RCC_APB2Periph_GPIOA
#define SPI_GPIO_SPI		GPIOA
#define SPI_Pin_SCK			GPIO_Pin_5
#define SPI_Pin_MISO		GPIO_Pin_6
#define SPI_Pin_MOSI		GPIO_Pin_7

#define SPI_GPIO_CE			GPIOB
#define SPI_Pin_CE      GPIO_Pin_2

#define SPI_GPIO_CSN		GPIOC
#define SPI_Pin_CSN			GPIO_Pin_15

void Delay(vu32 nCount);
void cycleCounterInit(void);
void SysTick_IRQ(void);

#endif 
