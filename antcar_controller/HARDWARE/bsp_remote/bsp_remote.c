#include "bsp_remote.h"
uint8_t Buff_Rx_Final[30] = {0};
uint8_t MessageRevFlag = 0;
uint8_t Remote_Rx_Buffer[30];
int8_t MessageFlag = -1;

void bsp_remote_USART_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(Remote_RCC_APB2Periph_USARTx|Remote_RCC_APB2Periph_GPIOx, ENABLE);
	
	//ï¿½ï¿½ï¿½ï¿½Txï¿½ï¿½ï¿½Å³ï¿½Ê¼ï¿½ï¿½
  GPIO_InitStructure.GPIO_Pin = Remote_GPIO_Pin_Tx;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(Remote_GPIO_Port, &GPIO_InitStructure);
	//ï¿½ï¿½ï¿½ï¿½Rxï¿½ï¿½ï¿½Å³ï¿½Ê¼ï¿½ï¿½
  GPIO_InitStructure.GPIO_Pin = Remote_GPIO_Pin_Rx;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(Remote_GPIO_Port, &GPIO_InitStructure); 
	//ï¿½ï¿½ï¿½ï¿½NVICï¿½ï¿½ï¿½ï¿½
	NVIC_InitStructure.NVIC_IRQChannel = Remote_USART_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  //USART ï¿½ï¿½Ê¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

  USART_Init(Remote_USARTx, &USART_InitStructure);
	//ï¿½ò¿ª´ï¿½ï¿½Ú½ï¿½ï¿½ï¿½ï¿½Ð¶Ï²ï¿½Ê¹ï¿½Ü´ï¿½ï¿½ï¿½
  USART_ITConfig(Remote_USARTx, USART_IT_RXNE, ENABLE);
  USART_Cmd(Remote_USARTx, ENABLE);  
}



int8_t SendData(uint8_t *SendBuff, uint8_t len)
{
	uint8_t i = 0;
	USART_ClearFlag(Remote_USARTx,USART_FLAG_TC);
	for( i=0;i<len;i++)
	{	
		USART_SendData(Remote_USARTx,SendBuff[i]);
		while (USART_GetFlagStatus(Remote_USARTx, USART_FLAG_TXE) == RESET);
	}
	return 1;
}

/***********************È«¾Ö±äÁ¿¶¨Òå½áÊø*********************************/
/*
 *¹¦ÄÜ£ºµÃµ½Ð£ÑéºÍ
 *²ÎÊý£ºdata-´«ÈëµÄÊý¾ÝÖ¸Õë£¬len-Êý¾Ý³¤¶È
 *·µ»Ø£ºcheck_sum ·µ»ØÐ£ÑéºÍ
 */
static uint16_t GetCheckSumRemote(uint8_t* DateInRemote,uint8_t lenInRemote)
{
	uint8_t i=0;
	uint16_t CheckSum = 0;
	for(i=0;i<lenInRemote;i++)
	{
		CheckSum += DateInRemote[(i + 4)]; 
	}
	return CheckSum;
}

u8 rx_stat=0, rx_cnt=0, frame_len=0;

void Remote_USARTx_IRQHandler(void)                	//ï¿½ï¿½ï¿½ï¿½1ï¿½Ð¶Ï·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
{
	uint16_t check_sum_cal = 0, check_sum_rcv = 0;
	uint8_t  rx_buf = 0;//,i = 0;	
	if(USART_GetFlagStatus(Remote_USARTx,USART_FLAG_ORE)==SET)
	{
		USART_ClearFlag(Remote_USARTx,USART_FLAG_ORE);
		USART_ReceiveData(Remote_USARTx);
		rx_cnt = 0;
		rx_stat = 0;
		return;
	}		
	while(USART_GetITStatus(Remote_USARTx, USART_IT_RXNE) != RESET)  //ï¿½ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½
	{
		
		rx_buf =USART_ReceiveData(Remote_USARTx);	//ï¿½ï¿½È¡ï¿½ï¿½ï¿½Õµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
		switch (rx_stat)
		{
					case 0:
					{
						rx_cnt = 0;
						rx_stat = 0;
						if(rx_buf == ((CarID >> 24) & 0xff))
						{
							
							Remote_Rx_Buffer[rx_cnt] = rx_buf;
							rx_cnt++;
							rx_stat=1;				
						}
//						else
//						{
//							rx_cnt = 0;
//							rx_stat = 0;
//						}
						break;
					}
					case 1:
					{
						if(rx_buf == ((CarID >> 16) & 0xff))
						{
							Remote_Rx_Buffer[rx_cnt] = rx_buf;
							rx_cnt++;
							rx_stat=2;
						}					
						else
						{
							rx_stat=0;
							rx_cnt=0;
						}
						break;					
					}
					case 2:
					{
						if(rx_buf == ((CarID >> 8) & 0xff))
						{
							Remote_Rx_Buffer[rx_cnt] = rx_buf;
							rx_cnt++;
							rx_stat = 3;					
						}
						else
						{
							rx_stat=0;
							rx_cnt=0;
						}
						break;						
					}
					case 3:
					{
						if(rx_buf == ((CarID >> 0) & 0xff))
						{
							Remote_Rx_Buffer[rx_cnt] = rx_buf;
							rx_cnt++;
							rx_stat = 4;							
						}
						else
						{
							rx_stat=0;
							rx_cnt=0;
						}
						break;						
					}
					case 4:
					{
						Remote_Rx_Buffer[rx_cnt] = rx_buf;
						rx_cnt++;
						frame_len = rx_buf;//ï¿½ï¿½ï¿½ï¿½Ö¡ï¿½ï¿½ï¿½ï¿½
						rx_stat=5;
						if(frame_len==0)
						{
							rx_stat=0;
							rx_cnt=0;
						}
						break;						
					}
					case 5:
					{
						Remote_Rx_Buffer[rx_cnt] = rx_buf;
						rx_cnt++;
						if((frame_len + 4) <= rx_cnt)
							rx_stat=6;
//						if(rx_cnt>15)
//							rx_cnt = rx_cnt;
						break;						
					}
					case 6:
					{
						Remote_Rx_Buffer[rx_cnt] = rx_buf;//Ð£ï¿½ï¿½Î»ï¿½Äµï¿½ï¿½Ö½ï¿½
						rx_cnt++;
						rx_stat=7;
						break;						
					}
					case 7:
					{
						Remote_Rx_Buffer[rx_cnt] = rx_buf;//Ð£ï¿½ï¿½Î»ï¿½Ä¸ï¿½ï¿½Ö½Ú£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

						check_sum_cal = GetCheckSumRemote(Remote_Rx_Buffer,frame_len);
						
						check_sum_rcv =(uint16_t)(Remote_Rx_Buffer[(Remote_Rx_Buffer[4] + 4)] | ((Remote_Rx_Buffer[(Remote_Rx_Buffer[4] + 5)]) << 8));
						
						if(check_sum_cal != check_sum_rcv)
						{
							rx_stat=0;
							rx_cnt = 0;	
						}
						else
						{
//							for(i=0;i<(rx_cnt+1);i++)
//								Buff_Rx_Final[i] = Remote_Rx_Buffer[i];	
							ReceiveDataAnalysis(Remote_Rx_Buffer);
						}
						rx_stat=0;
						rx_cnt = 0;
						break;						
					}				
					default:
					{
						rx_stat=0;
						rx_cnt = 0;	
						break;
					}
		}	
		USART_ClearITPendingBit(Remote_USARTx, USART_IT_RXNE);		
   }
	
}













