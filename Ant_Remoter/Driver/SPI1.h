#ifndef __SPI1_H__
#define __SPI1_H__

#include "stm32f10x.h"
	void SPI_Init(void);
	u8 SPI_RW(u8 dat);
	void SPI_CE_H(void);
	void SPI_CE_L(void);
	void SPI_CSN_H(void);
	void SPI_CSN_L(void);
	void SPI_SetSpeed(u8 SpeedSet);
	u8 SPI_ReadWriteByte(u8 TxData);
#endif










