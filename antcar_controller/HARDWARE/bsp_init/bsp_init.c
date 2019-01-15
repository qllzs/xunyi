#include "bsp_init.h"
/*
��ʱ������
tim2--->ϵͳ��ʱ��
tim5--->���Ź���ʱ��
tim3--->���Ŷ�ʱ��
tim4--->������ʱ��ʱ��


���ȼ�����  
����ͨ��---->�����ȼ�1   �����ȼ�1
���Ŷ�ʱ��-->�����ȼ�2   �����ȼ�2
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
	/*----------------ѡ��������-----------------*/
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
//	SD_Status = SD_Init();//���ļ�ϵͳ�л��г�ʼ������
//  SDOK = FsInit();
	E2PROMInit();
	#ifndef Debug_CCD
	printf("\r\nSD����ʼ�����\r\n");
	
	printf("ϵͳ��ʼ�����\r\n");
	#endif
	#ifndef BareDebug
	IWDG_Config(IWDG_Prescaler_64 ,625);
	#endif
}









































