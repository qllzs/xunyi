#include "bsp_init.h"

void BspInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	delay_init();	   	 	//��ʱ��ʼ�� 
	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	CanModeInit();
	System_TimeInit(1000); 	//ϵͳ�Ĳ�������
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
