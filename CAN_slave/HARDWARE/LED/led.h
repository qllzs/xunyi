#ifndef __LED_H
#define __LED_H	 
#include "GlobalParam.h" 

#define LED0 PBout(5)// PB5
#define LEDR PEout(12)// PE12	
#define LEDG PEout(10)// PE10	
#define LEDB PEout(11)// PE11	

#define LEDR_OFF  GPIO_SetBits(GPIOE,GPIO_Pin_12)
#define LEDR_ON   GPIO_ResetBits(GPIOE,GPIO_Pin_12)

#define LEDG_OFF  GPIO_SetBits(GPIOE,GPIO_Pin_10)
#define LEDG_ON   GPIO_ResetBits(GPIOE,GPIO_Pin_10)

#define LEDB_OFF  GPIO_SetBits(GPIOE,GPIO_Pin_11)
#define LEDB_ON   GPIO_ResetBits(GPIOE,GPIO_Pin_11)

void LED_Init(void);//³õÊ¼»¯


//ºì
#define LED_RED  			LEDR_ON;LEDG_OFF;LEDB_OFF
//ÂÌ
#define LED_GREEN	 		LEDR_OFF;LEDG_ON;LEDB_OFF
//À¶
#define LED_BLUE			LEDR_OFF;LEDG_OFF;LEDB_ON
//»Æ(ºì+ÂÌ)	
#define LED_YELLOW    LEDR_ON;LEDG_ON;LEDB_OFF
//×Ï(ºì+À¶)
#define LED_PURPLE    LEDR_ON;LEDG_OFF;LEDB_ON
//Çà(ÂÌ+À¶)
#define LED_CYAN      LEDR_OFF;LEDG_ON;LEDB_ON
//°×(ºì+ÂÌ+À¶)
#define LED_WHITE     LEDR_ON;LEDG_ON;LEDB_ON
//ºÚ(È«²¿¹Ø±Õ)
#define LED_RGBOFF    LEDR_OFF;LEDG_OFF;LEDB_OFF


#endif
