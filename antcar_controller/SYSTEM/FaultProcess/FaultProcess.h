#ifndef __FAULTPROCESS_H__
#define __FAULTPROCESS_H__
#include "bsp_include.h"

//���ϼĴ���λ�궨��
#define MOTOR_BIT      (0xff<<0)
#define BATTERY_BIT    (1<<8)
#define COL_FW_BIT     (1<<9)
#define COL_BW_BIT     (1<<10)
#define PULSE_BIT      (1<<11)
#define BOX_BIT        (1<<12)
//9-15λ����

#define FAULTDLY        200  //��ʱ�����е�ʱ��
/*----------------�ⲿ��������--------------------*/

extern volatile uint16_t FaultReg;
/*----------------�ⲿ��������--------------------*/

void FaultProcess(uint16_t FaultReg);

#endif  //__FAULTPROCESS_H__
