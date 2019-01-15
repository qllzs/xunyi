#ifndef __GLOBALPARAM_H
#define __GLOBALPARAM_H	
#include "stdio.h"	
#include "sys.h"
#include "eeprom_i2c.h"
#include "delay.h"
#include "usart.h"
#include "sys_time.h"
#include "can.h"
#include "led.h"
#include "boxmoter.h"
#include "ccd.h"
#include "bsp_init.h"
#include "bumper.h"
#include "watchdog.h"
#include "string.h"
#include "expandboard.h"

#define		UseWacthDog 	//使用独立看门狗	
/*********程序对应的模块定义**************/
#define		LinePatrolModule   //巡线模块程序

#define		xBoxMotorModule     //箱门电机程序

#define		xIOExtendModule    //IO扩展板程序

#define		xBumperModule	  //防撞模块
/**************End******************/

//巡线最大线宽
extern uint8_t 		MaxLinWidth;
//巡线最小线宽
extern uint8_t		MinLinWidth;
//黑线从无到有需要确认的次数
extern uint8_t		No2YsLineForgiveTime;
//黑线从有到无需要确认的次数
extern uint8_t 		Ys2NoLineForgiveTime;
//轴距
extern uint16_t		WheelBase;
//车轮直径
extern uint16_t 	VehRadii;
//箱门开启的速度(占空比)
extern uint16_t		BoxMotorDutyCycle;
//停止线检测是空心
extern uint8_t		StopLineHollow;
//箱门的状态
extern uint8_t		BoxReg;
extern uint8_t		BumperReg;


#endif
