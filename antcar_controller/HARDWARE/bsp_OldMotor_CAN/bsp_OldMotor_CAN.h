#ifndef __bsp_WheelMotor_CAN_H
#define __bsp_WheelMotor_CAN_H

#include "bsp_include.h"

#define ID1  0X01
#define ID2  0X02
#define ID3  0X03
#define ID4  0X04

#define KP   8000
#define KI   1000
#define KD   100

#define CAN_RX0_INT_ENABLE  1    // 使用中断接收数据

#define CANx                      CAN1
#define CAN_RX_PIN                GPIO_Pin_11
#define CAN_TX_PIN                GPIO_Pin_12
#define CAN_TX_GPIO_PORT          GPIOA
#define CAN_RX_GPIO_PORT          GPIOA
#define CAN_GPIO_CLK              RCC_APB2Periph_GPIOA
#define CAN_GPIO_AF_CLK           RCC_APB2Periph_AFIO
#define CAN_CLK                   RCC_APB1Periph_CAN1

#define OVERCURRENT               2400
extern uint8_t MotorFault;   // 电机故障标志

/* CAN函数声明 */

void CAN1_SendMesg(uint32_t id, uint8_t len, uint8_t *dat);

void Clear_RxMes(CanRxMsg *RxMessage);
/* 电机驱动指令函数声明 */
void Wheel_MOTOR_Init(void);
void MOTOR_STOP(void);
void EraseFault(void);
/* 电机控制相关函数声明 */  
void SingleMotorDeal(void);
void RTvelandang(void);
void ClearFault_EnableAgain(void);
/* CAN和电机总初始化及轮毂电机任务函数声明 */
void Motor_CAN_Init(void);
void SingleMotorDisable(uint32_t id);
#endif
















