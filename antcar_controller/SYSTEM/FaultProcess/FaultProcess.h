#ifndef __FAULTPROCESS_H__
#define __FAULTPROCESS_H__
#include "bsp_include.h"

//故障寄存器位宏定义
#define MOTOR_BIT      (0xff<<0)
#define BATTERY_BIT    (1<<8)
#define COL_FW_BIT     (1<<9)
#define COL_BW_BIT     (1<<10)
#define PULSE_BIT      (1<<11)
#define BOX_BIT        (1<<12)
//9-15位待定

#define FAULTDLY        200  //延时函数中的时间
/*----------------外部变量声明--------------------*/

extern volatile uint16_t FaultReg;
/*----------------外部函数声明--------------------*/

void FaultProcess(uint16_t FaultReg);

#endif  //__FAULTPROCESS_H__
