#ifndef __BSP_CCD_ADC_H__
#define __BSP_CCD_ADC_H__
#include "bsp_include.h"
/*-------------------------------TSL引脚宏定义-------------------------------*/
#define CCD_RCC_APBxPeriph_GPIOy  RCC_APB2Periph_GPIOA
#define TSL_SI_PORT               GPIOA
#define TSL_SI_Pin                GPIO_Pin_7

#define TSL_CLK_PORT              GPIOA
#define TSL_CLK_Pin               GPIO_Pin_6
/*-------------------------------TSL引脚电平宏定义-------------------------------*/
#define TSL_SI_H    GPIO_SetBits(TSL_SI_PORT, TSL_SI_Pin)   //SI  7
#define TSL_SI_L    GPIO_ResetBits(TSL_SI_PORT, TSL_SI_Pin) //SI  7

#define TSL_CLK_H   GPIO_SetBits(TSL_CLK_PORT, TSL_CLK_Pin)    //CLK 2
#define TSL_CLK_L   GPIO_ResetBits(TSL_CLK_PORT, TSL_CLK_Pin)  //CLK 2
/*------------------------------线性CCD所使用到的ADC宏定义--------------*/
#define CCD_ADC_GPIO_CLK          RCC_APB2Periph_GPIOA
#define CCD_RCC_APBxPeriph_ADCy   RCC_APB2Periph_ADC2

#define CCD_ADCx                  ADC2
#define CCD_ADC_CHANAL            ADC_Channel_5 
#define CCD_GPIO_PIN              GPIO_Pin_5
#define CCD_GPIO_PORT             GPIOA

#define NORMALWIDTH  26  // 根据CCD安装位置 实际测量采样点
#define K 30
#define	CCDMaxMinDiff		1000
#define	DiffTick(a,b)  ((a > b) ? (a - b) : (b-a))
#define PIX_START               	5       //显示范围
#define PIX_END		                123
#define PIX_NUM		                (PIX_END-PIX_START)
#define INT_TIME_MAX               20
#define ThreLineDiff								5
#define ThrePostDiff								10
#define	ThreWidthJump								40
#define	ThreWidthJumpCnt						5
#define	ThreFindNOLineCnt						30
#define	ThreFindHaveLineCnt					6
#define	CCDUpTime										2
#define	CCDDownTime									2
#define	BestAVG											2500
#define Kp													0.001
#define	Kd													0.01
#define	LeftThred										49
#define	RighThred										69
typedef struct _line
{
	uint8_t	left;
	uint8_t right;
	uint8_t	width;
}Str_Line;

extern u16 PixelValue_CCD[128];
extern int8_t IntegrationTime;
/*----------------------------外部函数声明----------------*/
void  CCD_Init(void);
//void  Find_CCD_Center_Width(uint8_t *LineCenter,uint8_t *LineWidth,uint16_t *PixelValue_CCD,volatile uint8_t *Line_Sta);
void Find_CCD_Center_Width(void* addr[6]);
void debug_ccd(uint16_t * ADV);
float turn(u8 CCD_zhongzhi);
void RD_TSL(uint16_t *ADV) ;
void Integration(void);
void CalculateIntegrationTime(uint16_t *CCD_AD,int8_t *IntTime);
#endif  //__BSP_CCD_ADC_H__

