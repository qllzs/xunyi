#ifndef __WATCHDOG_H
#define	__WATCHDOG_H
#include "GlobalParam.h"
void IWDG_Init(uint8_t prer,uint16_t reload);
void IWDG_Feed(void);
#endif
