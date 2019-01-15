#include "GlobalParam.h"

uint8_t FinRecvFlag=0; //接收数据完成标识
uint8_t BeginUpdate=0;
uint8_t	DoActionFlag=0;
//u16 FlashReadBuf[USART_REC_LEN/2];
extern uint16_t FrameLen;
extern StrCanRecvData	CanRecvData;
extern uint32_t time_100ms;
uint16_t ReadFrameLen=0;
uint8_t	testdata[8]={0x55,0xaa,0x01};
uint8_t	cnt=0;
int main(void)
{		
	BspInit();
	

	//BoxMotorRev(BoxMotorSpd);
	while(1)
	{
		#ifdef UseWacthDog
		IWDG_Feed();
		#endif
		if(DoActionFlag)
		{
			DoActionFunc(&CanRecvData);
			DoActionFlag = 0;
		}
		if((time_100ms%10) == 0)
		{
			time_100ms = 0;
			cnt = (++cnt)%2;
			if(cnt)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_8);
			}
			else
			{
				GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				//BoxMotorFwd(BoxMotorSpd);
			}

			//CAN1_SendMesg(((FuncM2SlavCmd<<7) | AddrMotorModul),8,testdata);
		}
	}
}














