#ifndef __BSP_INCLUDE_H__
#define __BSP_INCLUDE_H__


#include "debug_include.h"
#include "stm32f10x.h"

#include "protocol.h"
#include <string.h>
#include <math.h>
#include "system_timer.h"
#include "SysTick.h"

#include "bsp_init.h"
#include "bsp_battery_adc.h"
#include "bsp_wheel_motor.h"
#include "bsp_remote.h"
#include "bsp_anti_col.h"
#include "bsp_box_motor_timer.h"
#include "bsp_led.h"
#include "bsp_trail.h"
#include "bsp_eeprom_i2c.h"
#include "bsp_CCD_ADC.h"
#include "watchdog.h"
#include "FaultProcess.h"

#include <stdio.h>
#include "sdio_test.h"
#include "bsp_sdio_sdcard.h"
#include "ff.h"
#include "FileReadWrite.h"
#include <stdio.h>
#include "sdio_test.h"
#include "debug.h"
#include "FileReadWrite.h"
#ifdef METEC_MOTOR
#include "bsp_WheelMotor_CAN.h"
#endif  //METEC_MOTOR

#ifdef OLD_WHEEL_MOTOR
#include "bsp_OldMotor_CAN.h"
#endif  //OLD_WHEEL_MOTOR


extern volatile uint8_t SDOK;  //0:OK 1:Not OK
extern volatile uint8_t UpDownChangeMode;
extern FIL fnew;
extern uint8_t FindLineStatu;
#endif //__BSP_INCLUDE_H__

































