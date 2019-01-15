#include "GlobalParam.h" 
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
extern u8 FinRecvFlag,BeginUpdate;
	  

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 0
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 


//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
uint8_t USART1_RX_BUF[USART_REC_LEN]={0};
uint8_t USART2_RX_BUF[USART_REC_LEN]={0};

void uart_init(uint32_t bound1)
{
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	/**************************������115200*******************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	//USART3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB.10
   
  //USART3_RX	  GPIOB.11��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB.11  

  //Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound1;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;//����ģʽ

	USART_Init(USART3, &USART_InitStructure); //��ʼ������1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���1 
	/**********************�������************************************************/
	

}

uint8_t rx_statu=0;
uint8_t upmodule=0;
uint16_t FrameLen=0,rx_cnt=0;
void USART3_IRQHandler(void)
{
	u8 res;	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//���յ�����
	{	 
		res=USART_ReceiveData(USART3); 
		switch(rx_statu)
		{
			case 0:
				if(res == 0x55)
				{
					rx_statu++;
				}
			break;
			case 1:
				if(res == 0xAA)
				{
					rx_statu++;
				}
				else
				{
					rx_statu = 0;
				}
			break;
			case 2:
				if((res == 0x02) && (FinRecvFlag == 0))
				{
					rx_statu++;				
				}
				else
					rx_statu = 0;
			break;
			case 3:
				switch(res)
				{
					case 0x00:
						upmodule = 0;
						rx_statu++;
					break;
					case 0x01:
						upmodule = 1;
						rx_statu++;
					break;
					case 0x02:
						upmodule = 2;
						rx_statu++;
					break;
					default:
						upmodule = 0;
						rx_statu =0;
					break;
				}
			break;
			case 4:
				if(rx_cnt == 0)
				{
					FrameLen = (res & 0xff) << 8;
					rx_cnt++;
				}
				else if(rx_cnt == 1)
				{
					FrameLen |= (res & 0xff);
					if(rx_cnt < USART_REC_LEN)
					{
						rx_cnt=0;
						BeginUpdate = 1;
						rx_statu++;
					}
					else
					{
						rx_cnt=0;
						rx_statu=0;
					}
				}		
			break;
			case 5:
				USART1_RX_BUF[rx_cnt++] = res;
				if(rx_cnt >= FrameLen)
				{
					FinRecvFlag = 1;
					rx_statu = 0;
				}
			break;
		}
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	}
} 




