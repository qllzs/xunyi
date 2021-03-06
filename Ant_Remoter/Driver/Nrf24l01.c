#include "board.h"

//*********************************************NRF24L01*************************************
u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //本地地址
u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //接受地址

#define RX_DR				6		//中断标志
#define TX_DS				5
#define MAX_RT			4


uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uint8_t NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据
/*
*****************************************************************
* 写寄存器
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	SPI_CSN_L();					  /* 选通器件 */
	status = SPI_RW(reg);  /* 写寄存器地址 */
	SPI_RW(value);		  /* 写数据 */
	SPI_CSN_H();					  /* 禁止该器件 */
  return 	status;
}

/*
*****************************************************************
* 读寄存器
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI_CSN_L();					  /* 选通器件 */
	SPI_RW(reg);			  /* 写寄存器地址 */
	reg_val = SPI_RW(0);	  /* 读取该寄存器返回数据 */
	SPI_CSN_H();					  /* 禁止该器件 */
   return 	reg_val;
}
	
/*
*****************************************************************
*
* 写缓冲区
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
 {
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();				        /* 选通器件 */
	status = SPI_RW(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		SPI_RW(pBuf[i]);		/* 写数据 */
	}
	SPI_CSN_H();						/* 禁止该器件 */
    return 	status;	
}
/*
*****************************************************************
* 读缓冲区
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();						/* 选通器件 */
	status = SPI_RW(reg);	/* 写寄存器地址 */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = SPI_RW(0); /* 读取返回数据 */ 	
	}
	SPI_CSN_H();						/* 禁止该器件 */
    return 	status;
}

u8 NRF_RxPacket(uint8_t *rx_buf)
{
	u8 sta;		    							    
	sta=NRF_Read_Reg(NRFRegSTATUS);  //读取状态寄存器的值    	 
	NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF_Read_Buf(RD_RX_PLOAD,rx_buf,RX_PLOAD_WIDTH);//读取数据
		NRF_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器 	
		return 0;
	}	   
	return 1;
}



u8 NRF_TxPacket(uint8_t *tx_buf)
{		
	u8 sta=0;   //寄存器调试代码用变量
	
	SPI_CE_L();		 //StandBy I模式	
	NRF_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // 装载数据	
	SPI_CE_H();		 //置高CE，激发数据发送
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF_Read_Reg(NRFRegSTATUS);  //读取状态寄存器的值		
	NRF_Write_Reg(NRF_WRITE_REG+NRFRegSTATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}



//void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
//{	
//	//ANO_SPI_CE_L();		 //StandBy I模式	
//	ANO_NRF_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
//	//ANO_SPI_CE_H();		 //置高CE
//}



void NRF_TX_MODE(int *channel)
{

	SPI_CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址  
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址,使能ACK
	
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 0X01
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 0X01
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);						            					//设置自动重发间隔时间:500us;最大自动重发次数:10次 2M波特率下 0X1a
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,*channel);														//设置RF通道为CHANAL   必须一致
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//设置TX发射参数,0db增益,2Mbps,低噪声增益开启 必须一致
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送 	
	SPI_CE_H();
}

void NRF_RX_MODE(int *channel)
{
	SPI_CE_L(); 
	
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址,使能ACK
	
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//使能通道0的自动应答 0X01
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//使能通道0的接收地址 0X01
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,*channel);														//设置RF通道为CHANAL   必须一致
	NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//设置TX发射参数,0db增益,2Mbps,低噪声增益开启 必须一致
	NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
	SPI_CE_H();
}



uint8_t NRF_Check(void)
{ 
u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;
	//SPI1_SetSpeed(SPI_BaudRatePrescaler_8); //spi速度为9Mhz（24L01的最大SPI时钟为10Mhz）   	 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF_Read_Buf(TX_ADDR,buf,5); //读出写入的地址  
	
	for(i=0;i<5;i++)
		if(buf[i]!=0XA5)
			break;	 						
		
	if(i!=5)
		return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}







/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
