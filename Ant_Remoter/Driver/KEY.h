#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

#define KEY0  PBin(12)
#define KEY1  PBin(14)

#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 

void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(u8 mode);  	//����ɨ�躯��					    
#endif
