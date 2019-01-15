#include "bsp_include.h"
uint8_t Buffer_BlockTx[512] ={1,2,3};
uint8_t Buffer_BlockRx[512] ={0};
uint16_t num = 0;
int8_t IntegrationTime = 1;
uint8_t time_CCD=0,integration_point=0,linecnt=0,linegap=0;
uint32_t	ForwDriveDistance=0,RevDriveDistance=0; //正向行驶距离,逆向行驶距离
uint8_t LineWidth = 0,LinePosition = 0;
void *FindCCDAddr[6],*SpdSelctAddr[7];
uint8_t test[11]={0};
uint8_t String[11]={0};
uint8_t ret=0,ret1=0;
int main(void)
{ 
	
	
	HardwareInit();
  SDOK = f_open(&fnew, "0:CCData.txt", FA_OPEN_ALWAYS | FA_WRITE );
	while(1)
	{
		
		if(InstructionDoOverFlag)
		{
			InstructionDoOverFlag = Do_Instruction(Instruction_Structure);
		}	

	
		if((RevDriveDistance >= (ForwDriveDistance+25)) && (ModeReg == MOD_LINE_BACKFOLLOW))
		{
			ModeReg = 0;  //notice
			Vel_Set = 0;
			Ang_Set = 0;
			ForwDriveDistance=0;
			RevDriveDistance=0;
		}
	
		if(time_1ms > 1)
		{
			time_1ms = 0;
		}
		
		#ifdef AUTOMATIC_EXPLORE
		integration_point = INT_TIME_MAX - IntegrationTime;
		if(integration_point >= 2)
		{   /* 曝光点小于2(曝光时间大于18ms)则不进行再曝光 */
			if(integration_point == time_20ms)
			{
				Integration();           /* 曝光开始 */	
				if(integration_point == INT_TIME_MAX)
				{
					__set_PRIMASK(1);

					delay_us(50);
					RD_TSL(PixelValue_CCD);	
					time_100ms=0;
					CalculateIntegrationTime(PixelValue_CCD,&IntegrationTime);
					__set_PRIMASK(0);
					FindCCDAddr[0] = &LinePosition;
					FindCCDAddr[1] = &LineWidth;
					FindCCDAddr[2] = PixelValue_CCD;
					FindCCDAddr[3] = (void *)(&_Line_Sta);
					FindCCDAddr[4] = &linecnt;
					FindCCDAddr[5] = &linegap;
					Find_CCD_Center_Width(FindCCDAddr);
					
					//	Find_CCD_Center_Width(&LinePosition,&LineWidth,PixelValue_CCD,&_Line_Sta);
					
					SpdSelctAddr[0] = (void *)(&ModeReg);
					SpdSelctAddr[1] = &linecnt;
					SpdSelctAddr[2] = &linegap;
					SpdSelctAddr[3] = &VelOfCar;
					SpdSelctAddr[4] = &AngOfCar;
					SpdSelctAddr[5] = &LinePosition;
					SpdSelctAddr[6] = &LineWidth;
					CarSpeedSelection(SpdSelctAddr); 
					//CarSpeedSelection(ModeReg, &VelOfCar,&AngOfCar); 
					CarSpeedSet(VelOfCar,AngOfCar);
					
				}
			}
		}
		
	#endif
		
		if(time_20ms >= 20)
		{
			time_20ms = 0;
		 	
			#ifdef AUTOMATIC_EXPLORE		
				__set_PRIMASK(1);			
				RD_TSL(PixelValue_CCD);
				__set_PRIMASK(0);
				CalculateIntegrationTime(PixelValue_CCD,&IntegrationTime);

		#else
	
    		RD_TSL(PixelValue_CCD);
		#endif	
			FindCCDAddr[0] = &LinePosition;
			FindCCDAddr[1] = &LineWidth;
			FindCCDAddr[2] = PixelValue_CCD;
			FindCCDAddr[3] = (void *)(&_Line_Sta);
			FindCCDAddr[4] = &linecnt;
			FindCCDAddr[5] = &linegap;
			Find_CCD_Center_Width(FindCCDAddr);
			//Find_CCD_Center_Width(&LinePosition,&LineWidth,PixelValue_CCD,&_Line_Sta);
			
			SpdSelctAddr[0] = (void *)(&ModeReg);
			SpdSelctAddr[1] = &linecnt;
			SpdSelctAddr[2] = &linegap;
			SpdSelctAddr[3] = &VelOfCar;
			SpdSelctAddr[4] = &AngOfCar;
			SpdSelctAddr[5] = &LinePosition;
			SpdSelctAddr[6] = &LineWidth;
			CarSpeedSelection(SpdSelctAddr); 
			
			//CarSpeedSelection(ModeReg, &VelOfCar,&AngOfCar); 
			CarSpeedSet(VelOfCar,AngOfCar);
			    
		}
		if(time_10ms >= 10)
		{
			time_10ms = 0;
			FaultProcess(FaultReg);
			#ifdef METEC_MOTOR
				CarSpeedRead(&SpeedFeedBack_Structure.velocity, &SpeedFeedBack_Structure.angular);				
			#else
				RTvelandang();				
			#endif
			if(ModeReg == MOD_LINE_FOLLOW)
			{
				if(SpeedFeedBack_Structure.velocity < 0)
					ForwDriveDistance += (-SpeedFeedBack_Structure.velocity)*100/100; //单位0.1mm
				else
					ForwDriveDistance += (SpeedFeedBack_Structure.velocity)*100/100;
			}
			else if(ModeReg == MOD_LINE_BACKFOLLOW)
			{
				if(SpeedFeedBack_Structure.velocity < 0)
					RevDriveDistance += (-SpeedFeedBack_Structure.velocity)*100/100; //单位0.1mm
				else
					RevDriveDistance += (SpeedFeedBack_Structure.velocity)*100/100;
			}
			SendSpeedMessageToHost(SpeedFeedBack_Structure.velocity , SpeedFeedBack_Structure.angular); 
			
		}
		
		


		if(time_50ms >= 50)
		{
			time_50ms = 0;
			

		}
		if(time_100ms >= 100)
		{
			time_100ms = 0;		
		}
		if(time_200ms >= 200)
		{
			time_200ms = 0;
			#ifndef BareDebug
			IWDG_Feed();//喂狗
			#endif
			#ifdef Debug_CCD		
			debug_ccd(PixelValue_CCD);
			#endif
			SendPulseMessageToHost(TimeStampReg,ModeReg, BoxReg ,_Line_Sta);
		}
		if(time_1000ms >= 1000)
		{
			time_1000ms = 0;
			
			
			
			BatteryVoltage = ReadBatteryVoltage( &FaultReg,&BatteryRemainder);
//			sprintf(WriteBuffer,"\n电池电压为：%d",BatteryVoltage);
//			fileWrite(WriteBuffer,"0:电池电压数据.txt");
			CarMotorFaultCheck( &FaultReg );
			SendVoltageMessageToHost(TimeStampReg,BatteryVoltage,0,BatteryRemainder,0);
		}
		if(time_5000ms >= 5000)
		{
			if(SDOK)
			{
				f_close(&fnew);
			}
			else
			{
				SDOK = f_close(&fnew);
				SDOK = f_open(&fnew, "0:CCData.txt", FA_OPEN_ALWAYS | FA_WRITE );
			}
		}
	}	 
}


