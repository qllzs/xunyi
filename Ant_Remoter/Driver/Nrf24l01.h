#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include "board.h"


//***************************************NRF24L01寄存器指令*******************************************************
#define NRF_READ_REG        0x00  	// 读寄存器指令
#define NRF_WRITE_REG       0x20 	// 写寄存器指令
#define R_RX_PL_WID   	0x60

#define RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define FLUSH_TX        0xE1 	// 冲洗发送 FIFO指令
#define FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NOP             0xFF  	// 保留

//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define NRFRegSTATUS    0x07  // 状态寄存器

#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道1接收数据长度
#define RX_PW_P2        0x13  // 接收频道2接收数据长度
#define RX_PW_P3        0x14  // 接收频道3接收数据长度
#define RX_PW_P4        0x15  // 接收频道4接收数据长度
#define RX_PW_P5        0x16  // 接收频道5接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//**************************************************************************************


//24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5   	//5字节的地址宽度
#define RX_ADR_WIDTH    5   	//5字节的地址宽度
#define TX_PLOAD_WIDTH  8  	//32字节的用户数据宽度
#define RX_PLOAD_WIDTH  8  	//32字节的用户数据宽度

/*********************************************/
//24L01操作线
#define NRF24L01_CE   PBout(2) //24L01片选信号
#define NRF24L01_IRQ  PCin(14)  //IRQ主机数据输入
#define NRF24L01_CSN  PCout(15) //SPI片选信号	



	//用于初始化为发射模式
	void NRF_TX_MODE(int *channel);
	//用于初始化为接收模式
	void NRF_RX_MODE(int *channel);
	//发送数据包,用于model 2/4
	u8 NRF_TxPacket(uint8_t * tx_buf);	
	
	u8 NRF_RxPacket(uint8_t *rx_buf);
	//发送数据包,用于model 3
	void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len);	
	//检查NRF模块是否正常工作
	uint8_t NRF_Check(void);
	//检查是否有通信事件
	void NRF_Check_Event(void);

	extern uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
	extern uint8_t NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据
	
	
	uint8_t NRF_Read_Reg(uint8_t reg);
	uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
	uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
	uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
	
#endif















