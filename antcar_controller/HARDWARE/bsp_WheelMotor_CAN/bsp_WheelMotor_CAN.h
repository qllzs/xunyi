#ifndef __bsp_WheelMotor_CAN_H
#define __bsp_WheelMotor_CAN_H

#include "bsp_include.h"

//#define ID1  0X01
//#define ID2  0X02
//#define ID3  0X03
//#define ID4  0X04

//#define KP   8000
//#define KI   1000
//#define KD   100

#define CAN_RX0_INT_ENABLE  1    // 使用中断接收数据

#define CANx                      CAN1
#define CAN_RX_PIN                GPIO_Pin_11
#define CAN_TX_PIN                GPIO_Pin_12
#define CAN_TX_GPIO_PORT          GPIOA
#define CAN_RX_GPIO_PORT          GPIOA
#define CAN_GPIO_CLK              RCC_APB2Periph_GPIOA
#define CAN_GPIO_AF_CLK           RCC_APB2Periph_AFIO
#define CAN_CLK                   RCC_APB1Periph_CAN1 
/*-------------------CAN电机编号宏定义------------------------*/
#define MOTOR_L                     2
#define MOTOR_R                     1


extern int32_t Speed_L_RPM,Speed_R_RPM;
/*--------------------命令ID宏定义-------------------------*/
#define  PRIM_ENABLE               0X15
#define  PRIM_SETVELOCITY          0X6F
#define  PRIM_DISABLE              0X16
#define  PRIM_BREAKEOPERATION      0X1E
#define  PRIM_GETSERIALNO          0X08
#define  PRIM_GETACTVELOCITY       0X3F    //获取速度
#define  PRIM_GETERROR             0X3D    //获取故障类型


/*---------------------故障类型宏定义------------------------*/
#define FAULT_SYSTEM_ERROR      (0X1<<0)   //故障编号1程序运行故障 不可清除
#define FAULT_UNDER_VOLTAGE     (0X1<<3)   //故障编号4欠压故障 可清除
#define FAULT_I2T_ERROR         (0X1<<5)   //故障编号6过载故障 可清除

/*----------------------外部函数声明-----------------------------------*/
void SendMessage2MotorDriver(uint8_t Ad,uint8_t CmdID,int16_t param1,int16_t param2);
void Metec_MOTOR_Init(void);

//#define OVERCURRENT               2400
//extern uint8_t MotorFault;   // 电机故障标志
/* 电机驱动指令函数声明 */
//void Wheel_MOTOR_Init(void);
//void MOTOR_STOP(void);
//void EraseFault(void);
/* 电机控制相关函数声明 */
//void controlmessage(float vel, float ang);  
//void SingleMotorDeal(void);
//void RTvelandang(void);
//void ClearFault_EnableAgain(void);
/* CAN和电机总初始化及轮毂电机任务函数声明 */
//void Motor_CAN_Init(void);
//void WheelMotorCtrol_Task(void);
//void SingleMotorDisable(uint32_t id);
#endif

