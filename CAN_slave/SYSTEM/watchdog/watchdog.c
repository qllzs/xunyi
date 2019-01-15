#include "watchdog.h"

/*****************
*prer:��Ƶϵ��   reload: ��������
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
