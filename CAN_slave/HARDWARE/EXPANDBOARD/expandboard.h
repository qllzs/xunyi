#ifndef	__EXPANDBOARD_H
#define	__EXPANDBOARD_H
#include "GlobalParam.h"

#define	HBridgeDutyClc		500
/**********H桥配置**************/
//H桥PWM接口配置
#define RCC_EXPANDPWMClockCmd     		RCC_APB2Periph_TIM1
#define RCC_EXPANDPWMGPIOClockCmd		RCC_APB2Periph_GPIOB
#define	TIMEXPANDPWM					TIM1
#define TIMEXPANDPWMOCInit				TIM_OC3Init
#define TIMEXPANDPWMOCPreloadConfig		TIM_OC3PreloadConfig
#define EXPANDPWMGPIOPIN				GPIO_Pin_15
#define EXPANDPWMGPIOPORT				GPIOB
//H桥逻辑配置
#define	RCCHBridgeClockCmd				RCC_APB2Periph_GPIOB
#define	HBridgeOUTGPIOPIN				GPIO_Pin_14
#define	HBridgeOUTGPIOPORT				GPIOB
/***********************************/

/*************RGB 配置*********************/
#define	RCC_RGBGPIOClockCmd				RCC_APB2Periph_GPIOB
#define	BGPIOPIN						GPIO_Pin_13
#define	RGPIOPIN						GPIO_Pin_12
#define	GGPIOPIN						GPIO_Pin_7
#define	RGBGPIOPORT						GPIOB
/***************************************/

/************数字输出*********************/
#define RCCDigtalOUTGPIOClockCmd		(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC)
#define	D1GPIOPIN						GPIO_Pin_6
#define	D2GPIOPIN						GPIO_Pin_5
#define	D3GPIOPIN						GPIO_Pin_4
#define	D4GPIOPIN						GPIO_Pin_3
#define	D8GPIOPIN						GPIO_Pin_9
#define	DIGTALUPGPIOPORT				GPIOB

#define	DIGTALDOWNGPIOPORT				GPIOC
#define	D5GPIOPIN						GPIO_Pin_15
#define	D6GPIOPIN						GPIO_Pin_14
#define	D7GPIOPIN						GPIO_Pin_13
/******************************************/

/*************数字输入********************/
#define	RCC_DIGTALIN1to3GPIOClockCmd	(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO)
#define	RCC_DIGTALIN4GPIOClockCmd		RCC_APB2Periph_GPIOB
#define	DIGTALIN1to3GPIOPORT			GPIOA
#define	DIGTALIN4GPIOPORT				GPIOC
#define	DIN1GPIOPIN						GPIO_Pin_9
#define	DIN2GPIOPIN						GPIO_Pin_10
#define	DIN3GPIOPIN						GPIO_Pin_15
#define	DIN4GPIOPIN						GPIO_Pin_0
/*********************************************/

/********************模拟输入***************************/
#define  RCC_ANALOGIN    (RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA)
#define  ANALOGINPOART    GPIOA

/*******模拟电压输入***********/
#define  ADC1GPIO    GPIO_Pin_0
#define  ADC2GPIO    GPIO_Pin_1
#define  ADC3GPIO    GPIO_Pin_2
#define  ADC4GPIO    GPIO_Pin_3
/*******模拟电流输入************/
#define  VOA1GPIO    GPIO_Pin_4
#define  VOA2GPIO    GPIO_Pin_5
#define  VOA3GPIO    GPIO_Pin_6
#define  VOA4GPIO    GPIO_Pin_7
/********************************************************/

void DigtalOutHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DigtalOutLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DigtalOutPutAllLow(void);
void DigtalOutLow(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DigtalOutHigh(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void DigtalIOOutInit(void);
void IODigtalOut(void);
void RGBIOUTInit(void);
void HBridgePwmInit(void);
void HBridgeDriveFwd(void);
void HBridgeDriveRev(void);
void DigtalIOINInit(void);
void DigtalOutControl(uint8_t Channel);
uint8_t DigtalINControl(uint8_t Channel);
void ANALOG_IN_Init(void);
void AnalogVolageIN(void);
void AnalogCurrentIN(void);
#endif

