#ifndef __BSP_BATTERY_ADC_H__
#define __BSP_BATTERY_ADC_H__
#include "bsp_include.h"
/*---------------宏定义-----------------*/
#define ADC_PIN                GPIO_Pin_3
#define ADC_GPIO_PORT          GPIOC
#define ADC_GPIO_CLK           RCC_APB2Periph_GPIOC
#define RCC_APBxPeriph_ADCy    RCC_APB2Periph_ADC1

#define ADCx                  ADC1
#define ADC_Channel_x         ADC_Channel_13

//最大电池电量宏定义,单位0.01V
#define BASE_VOLTAGE       2220
//还可以在撑会，可以换电池了 30%
#define WARNING_VOLTAGE       2310

//临界电压模式，撑不多久了  15%
#define CRITICAL_VOLTAGE			2265

#define WINDOW_VALUE          15
#define	SoftFilterCnt					10

extern uint16_t BatteryVoltage,BatteryRemainder;
/*---------------外部函数声明-----------------*/
void ReadBatteryVoltageADC_Init(void);
uint16_t ReadBatteryVoltage(volatile uint16_t *FaultReg,volatile uint16_t *remaider);
#endif   //__BSP_BATTERY_ADC_H__












