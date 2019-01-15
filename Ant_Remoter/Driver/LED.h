#ifndef __ANO_DRV_LED_H__
#define __ANO_DRV_LED_H__
#include "board.h"

void LED_Init(void);
void LED_0_ON(void);
void ANO_LED_0_OFF(void);
void ANO_LED_1_ON(void);
void ANO_LED_1_OFF(void);

//指示灯
void LED_0_FLASH(void);
void LED_TX_BULING(void);  //发送模式指示灯
void LED_RX_BULING(void);  //接受模式指示灯
#endif

