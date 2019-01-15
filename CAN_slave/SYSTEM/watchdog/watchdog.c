#include "watchdog.h"

/*****************
*prer:分频系数   reload: 重载数据
***************************************/
void IWDG_Init(uint8_t prer,uint16_t reload)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(prer); 
	IWDG_SetReload(reload) ;
	IWDG_ReloadCounter(); 
	IWDG_Enable(); 
}

void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}
