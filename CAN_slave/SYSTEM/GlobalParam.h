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

#define		UseWacthDog 	//ʹ�ö������Ź�	
/*********�����Ӧ��ģ�鶨��**************/
#define		LinePatrolModule   //Ѳ��ģ�����

#define		xBoxMotorModule     //���ŵ������

#define		xIOExtendModule    //IO��չ�����

#define		xBumperModule	  //��ײģ��
/**************End******************/

//Ѳ������߿�
extern uint8_t 		MaxLinWidth;
//Ѳ����С�߿�
extern uint8_t		MinLinWidth;
//���ߴ��޵�����Ҫȷ�ϵĴ���
extern uint8_t		No2YsLineForgiveTime;
//���ߴ��е�����Ҫȷ�ϵĴ���
extern uint8_t 		Ys2NoLineForgiveTime;
//���
extern uint16_t		WheelBase;
//����ֱ��
extern uint16_t 	VehRadii;
//���ſ������ٶ�(ռ�ձ�)
extern uint16_t		BoxMotorDutyCycle;
//ֹͣ�߼���ǿ���
extern uint8_t		StopLineHollow;
//���ŵ�״̬
extern uint8_t		BoxReg;
extern uint8_t		BumperReg;


#endif
