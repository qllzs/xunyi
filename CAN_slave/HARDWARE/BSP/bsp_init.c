#include "bsp_init.h"

void BspInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	delay_init();	   	 	//延时初始化 
	LED_Init();		  		//初始化与LED连接的硬件接口
	CanModeInit();
	System_TimeInit(1000); 	//系统的步调函数
	ANALOG_IN_Init();
	#ifdef UseWacthDog
	IWDG_Init(4,625);
	#endif
	#ifdef BoxMotorModule
	uart_init(115200);
	BoxMotorInit();
	#elif defined BumperModule
	BumperInit();
	#elif defined IOExtendModule
	IODigtalOut();
	#endif
}
