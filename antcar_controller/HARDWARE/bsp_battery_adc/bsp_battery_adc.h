#ifndef __BSP_BATTERY_ADC_H__
#define __BSP_BATTERY_ADC_H__
#include "bsp_include.h"
/*---------------�궨��-----------------*/
#define ADC_PIN                GPIO_Pin_3
#define ADC_GPIO_PORT          GPIOC
#define ADC_GPIO_CLK           RCC_APB2Periph_GPIOC
#define RCC_APBxPeriph_ADCy    RCC_APB2Periph_ADC1

#define ADCx                  ADC1
#define ADC_Channel_x         ADC_Channel_13

//����ص����궨��,��λ0.01V
#define BASE_VOLTAGE       2220
//�������ڳŻᣬ���Ի������ 30%
#define WARNING_VOLTAGE       2310

//�ٽ��ѹģʽ���Ų������  15%
#define CRITICAL_VOLTAGE			2265

#define WINDOW_VALUE          15
#define	SoftFilterCnt					10

extern uint16_t BatteryVoltage,BatteryRemainder;
/*---------------�ⲿ��������-----------------*/
void ReadBatteryVoltageADC_Init(void);
uint16_t ReadBatteryVoltage(volatile uint16_t *FaultReg,volatile uint16_t *remaider);
#endif   //__BSP_BATTERY_ADC_H__












