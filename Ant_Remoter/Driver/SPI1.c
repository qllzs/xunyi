#include "board.h"
#include "SPI1.h"
#include "sys.h"
//#include "stm32f10x_pwr.h"
//#include "stm32f10x_bkp.h"

void SPI_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  R_SPI_InitTypeDef  R_SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE );	
	
	//PA5-SPI_SCLK PA6-SPI_MISO PA7-SPI_MOSI
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//NRF_CE   PB2   LCD_CS  PB9
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_9);

	//NRF_IRQ  PC14
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   //推挽输出
	GPIO_Init(GPIOC, &GPIO_InitStructure); 

	//NRF_CSN  PC15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
	GPIO_SetBits(GPIOC,GPIO_Pin_15);
	

	//设置nrf2401c需要的配置参数
	R_SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	R_SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	R_SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	R_SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//选择了串行时钟的稳态:时钟悬空低电平
	R_SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//数据捕获于第一个时钟沿
	R_SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	R_SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//定义波特率预分频的值:波特率预分频值为4
	R_SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	R_SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
	R_SPI_Init(SPI1, &R_SPI_InitStructure);  //根据R_SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	//SPI_RW(0xff);        //启动传输
}

u8 SPI_RW(u8 dat) 
{ 
	/* 当 SPI发送缓冲器非空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
	/* 通过 SPI2发送一字节数据 */ 
	SPI_I2S_SendData(SPI1, dat); 
	/* 当SPI接收缓冲器为空时等待 */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI1); 
}


void SPI_CSN_H(void)
{
	GPIO_SetBits(SPI_GPIO_CSN, SPI_Pin_CSN);
}

void SPI_CSN_L(void)
{
	GPIO_ResetBits(SPI_GPIO_CSN, SPI_Pin_CSN);
}

void SPI_CE_H(void)
{
	GPIO_SetBits(SPI_GPIO_CE, SPI_Pin_CE);
}

void SPI_CE_L(void)
{
	GPIO_ResetBits(SPI_GPIO_CE, SPI_Pin_CE);
}

//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
  
void SPI_SetSpeed(u8 SpeedSet)
{
	//R_SPI_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	//R_SPI_Init(SPI1, &R_SPI_InitStructure);
//	SPI_Cmd(SPI1,ENABLE);
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet;	//设置SPI1速度  
	SPI1->CR1|=1<<6; 		//SPI设备使能 
} 

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
