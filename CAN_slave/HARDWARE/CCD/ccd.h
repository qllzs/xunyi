#ifndef __CCD_H
#define __CCD_H
#include "GlobalParam.h"

#define TSL_SI_PORT               GPIOA
#define TSL_SI_Pin                GPIO_Pin_7

#define TSL_CLK_PORT              GPIOA
#define TSL_CLK_Pin               GPIO_Pin_6

#define TSL_SI_H    GPIO_SetBits(TSL_SI_PORT, TSL_SI_Pin)   //SI  ธ฿
#define TSL_SI_L    GPIO_ResetBits(TSL_SI_PORT, TSL_SI_Pin) //SI  ตอ

#define TSL_CLK_H   GPIO_SetBits(TSL_CLK_PORT, TSL_CLK_Pin)    //CLK  ธ฿
#define TSL_CLK_L   GPIO_ResetBits(TSL_CLK_PORT, TSL_CLK_Pin)  //CLK  ตอ

#define CCD_ADC                  	ADC2
#define CCD_ADC_CHANAL            ADC_Channel_5 
#define CCD_GPIO_PIN              GPIO_Pin_5
#define CCD_GPIO_PORT             GPIOA

void CCDInit(void);
uint16_t Get_CCD_PixelValue(void);
void delay_CCD(void);
void Integration(void);
void RD_TSL(uint16_t *ADV);
#endif
