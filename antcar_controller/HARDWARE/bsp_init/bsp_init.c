#include "bsp_init.h"
/*
定时器分配
tim2--->系统定时器
tim5--->看门狗定时器
tim3--->箱门定时器
tim4--->箱门延时定时器


优先级配置  
串口通信---->主优先级1   子优先级1
箱门定时器-->主优先级2   子优先级2
*/
volatile uint8_t SDOK=0;
void HardwareInit(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTickInit();
	CCD_Init();
	SYSTEM_TIM_Init(1000);
	WDG_TIM_Init(1000);
	bsp_remote_USART_Init(115200);
	ReadBatteryVoltageADC_Init();
	
	Box_Init();
	delay_ms(1000);	
	/*----------------选择电机种类-----------------*/
	#ifdef METEC_MOTOR
	Metec_MOTOR_Init();
	#endif
	
	#ifdef MOTOR_485
	bsp_WheelMotor_init();
	#endif
	
	#ifdef OLD_WHEEL_MOTOR
	Motor_CAN_Init();
	#endif	
//	delay_ms(5000);	
	/*----------------end-----------------*/
	LED_GPIO_Init();
	led_blink(0,5,ENABLE);
	//TrailGPIOInit();
	debug_usart();
	
	AntiCol_EXTI_IO_Init();
//	SD_Status = SD_Init();//在文件系统中会有初始化过程
//  SDOK = FsInit();
	E2PROMInit();
	#ifndef Debug_CCD
	printf("\r\nSD卡初始化完成\r\n");
	
	printf("系统初始化完成\r\n");
	#endif
	#ifndef BareDebug
	IWDG_Config(IWDG_Prescaler_64 ,625);
	#endif
}









































