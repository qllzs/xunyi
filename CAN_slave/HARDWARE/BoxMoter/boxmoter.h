#ifndef __BOXMOTER_H
#define __BOXMOTER_H
#include "GlobalParam.h"
//����������
#define		CmdBoxOff			0x00
//������
#define		CmdBoxOn			0x01
//�������ڶ���
#define		StaBoxIng			0x02
#define		BoxMotorSpd			500//����Խ��Խ��
/**********PWM��ʱ������***********/
#define RCC_PWM1ClockCmd     		RCC_APB2Periph_TIM1
#define RCC_PWM2ClockCmd     		RCC_APB2Periph_TIM1
#define RCC_PWM1GPIOClockCmd		RCC_APB2Periph_GPIOA
#define RCC_PWM2GPIOClockCmd		RCC_APB2Periph_GPIOA
#define	TIMPWM1						TIM1
#define	TIMPWM2						TIM1
#define TIMPWMOC1Init				TIM_OC2Init
#define TIMPWMOC2Init				TIM_OC3Init
#define TIM_PWM1OCPreloadConfig    	TIM_OC2PreloadConfig
#define TIM_PWM2OCPreloadConfig    	TIM_OC3PreloadConfig

#define PWM1GPIOPIN					GPIO_Pin_9
#define PWM2GPIOPIN					GPIO_Pin_10
#define PWM1GPIOPORT				GPIOA
#define PWM2GPIOPORT				GPIOA


/***************��λ��������***********************/
#define LIMIT_SWITCH_A_GPIO_PORT         GPIOA
#define LIMIT_SWITCH_A_GPIO_CLK          RCC_APB2Periph_GPIOA
#define LIMIT_SWITCH_A_GPIO_PIN          GPIO_Pin_6
#define LIMIT_SWITCH_A_EXTI_PORTSOURCE   GPIO_PortSourceGPIOA
#define LIMIT_SWITCH_A_EXTI_PINSOURCE    GPIO_PinSource6
#define LIMIT_SWITCH_A_EXTI_LINE         EXTI_Line6
#define LIMIT_SWITCH_A_EXTI_IRQ          EXTI9_5_IRQn
#define	LIMIT_SWITCH_AIRQ_Handler		 EXTI9_5_IRQHandler
#define LIMIT_SWITCH_AIRQ_READ()  		((LIMIT_SWITCH_A_GPIO_PORT->IDR & LIMIT_SWITCH_A_GPIO_PIN) != 0)	/* ��IRQ����״̬ */

#define LIMIT_SWITCH_B_GPIO_PORT         GPIOB
#define LIMIT_SWITCH_B_GPIO_CLK          RCC_APB2Periph_GPIOB
#define LIMIT_SWITCH_B_GPIO_PIN          GPIO_Pin_0
#define LIMIT_SWITCH_B_EXTI_PORTSOURCE   GPIO_PortSourceGPIOB
#define LIMIT_SWITCH_B_EXTI_PINSOURCE    GPIO_PinSource0
#define LIMIT_SWITCH_B_EXTI_LINE         EXTI_Line0
#define LIMIT_SWITCH_B_EXTI_IRQ          EXTI0_IRQn
#define	LIMIT_SWITCH_BIRQ_Handler		 EXTI0_IRQHandler
#define LIMIT_SWITCH_BIRQ_READ()  		((LIMIT_SWITCH_B_GPIO_PORT->IDR & LIMIT_SWITCH_B_GPIO_PIN) != 0)	/* ��IRQ����״̬ */
/****************��λ����********************/
/****************���ŵ���ʱ��������ֹ���ſ��������ж�ת*****************/
#define	BoxTime							 TIM3
#define BoxTimeIRQ						 TIM3_IRQn
#define	BoxTimeHandler					 TIM3_IRQHandler
#define Box_TIM_Prescaler         		 7200  // 0.1ms�ķ�Ƶ
#define	BoxTimeInitCnt					 1
#define	BoxTimeCnt						 150
#define	BoxDelayFreq					 1000   //100ms
	
extern  int	BoxDelayTime[BoxTimeInitCnt];

void	BoxMotorControl(uint8_t Cmd); //���ſ��ƶ���
void  	BoxStauts(uint8_t *Status);  //���ŵ�״̬  0���ر�;1������2�������м�״̬
void 	BoxMotorInit(void);
void 	BoxLimitSwitchConfig(void);
void 	BoxMotorStop(void);
void 	BoxMotorRev(uint16_t Speed);
void 	BoxMotorFwd(uint16_t Speed);
void 	BoxTimePwmInit(void);
void 	BoxTimeInit(void);
#endif

