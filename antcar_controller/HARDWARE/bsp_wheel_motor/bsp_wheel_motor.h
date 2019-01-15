#ifndef  __BSP_WHEEL_MOTOR_H__
#define  __BSP_WHEEL_MOTOR_H__
#include "bsp_include.h"


/********************���ں궨��**********************/
// ����2-USART2
#define  Motor_USARTx                   USART2
#define  Motor_USART_CLK                RCC_APB1Periph_USART2
#define  Motor_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  Motor_USART_BAUDRATE           38400

// USART GPIO ���ź궨��
#define  Motor_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  Motor_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  Motor_USART_TX_GPIO_PORT       GPIOA   
#define  Motor_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  Motor_USART_RX_GPIO_PORT       GPIOA
#define  Motor_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  Motor_USART_IRQ                USART2_IRQn
#define  Motor_USART_IRQHandler         USART2_IRQHandler
//DMA����ͨ�������궨��
#define DMA_Rx_BUFFER_SIZE              16
#define Motor_USARTx_DMA_Rx_Channel     DMA1_Channel6
#define RCC_AHBPeriph_DMA_USARTx_Motor   RCC_AHBPeriph_DMA1
//DMA����ͨ�������궨��
#define DMA_Tx_BUFFER_SIZE              8
#define Motor_USARTx_DMA_Tx_Channel     DMA1_Channel7
#define Motor_USARTx_DMA_FLAG_TC        DMA1_FLAG_TC7
/********************�������궨��**********************/
//������궨��
#define W_S_REG        0x06 //д�����Ĵ���
#define R_REG          0x03 //���Ĵ���


//��д�Ĵ�����ַ�궨��
#define CON_MOD_REG        0X0011   //����ģʽ0���ڲ��ٶ�ģʽ  1����λ��ģʽ
#define INTER_SPEED_REG    0X0012   //RPM
#define KP_SPEED_REG       0X0013   //�ٶ�pid����10-10000
#define KI_SPEED_REG       0X0014   //�ٶ�pid����10-10000
#define STOP_MODE_REG      0X0015   //ͣ����ʽ 0������ͣ�� 1��ɲ��ͣ��   2������ͣ��
#define BREAK_STRENTH_REG  0X0016   //ɲ��ͣ��ǿ��%
#define ACCELERATE_REG     0X0017   //���ٶ�rpm/ms
#define DECELERATE_REG     0X0018   //���ٶ�rpm/ms
#define GAIN_REG           0X0019   //��λ����������rpm/v
#define OVER_CURRENT_REG   0X001a   //���ص���ֵ A
#define OVER_TIME_REG      0X001b   //����ʱ��ms
#define ALARM_CURRENT_REG  0X001c   //����������ֵA
#define PWM_FREQUENCY_REG  0X001d   //PWM����Ƶ��4-16kHz
#define LIMIT_DUTY_REG     0X001e   //PWM�������ռ�ձ�10%-100%
#define MOTOR_POLE_REG     0X001f   //����ļ�����
#define X0_FUCTION_REG     0X0020   //����X0���ŵĹ��� 0��X0����Ϊɲ������  1��X0����Ϊ����ٹ���
#define INTER_SPEED2_REG   0X0021   //�ڶ��ڲ��ٶ�RPM
#define BAUDRATE_REG       0X0022   //ͨѶ������ 0:9600  1:38400   2:57600   3:115200
#define DRIVER_NUM_REG     0X0023   //���������1-255
#define IO_TOGGLE_REG      0X0024   //����I/O�߼�ȡ��1:001��runȡ���� 2:010��DIRȡ���� 4:100��breakȡ����
#define PARA_SAVE          0X0025   //��������Ĵ�����д1����
//ֻ��16λ�Ĵ�����ַ
#define CURRENT_SPEED_REG      0X0001   //RPM
#define HAL_STAT_REG           0X0002   //5-1-3-2-6-4
#define FAULT_STAT_REG         0X0003   //�ο����ϴ���
#define BUS_VOL_REG            0X0004   //V
#define CURRENT_REG            0X0005   //001A
#define DRIVER_TEMP_REG        0X0006   //��

//���ϴ���궨��
#define LOCKED_ALARM          0X0001   //�����ת����
#define OVER_CURRENT_ALARM    0X0002   //��������
#define HAL_FAULT_ALARM       0X0004   //�������ϱ���
#define VOL_DOWM_ALARM        0X0008   //Ƿѹ����
#define VOL_UP_ALARM          0X0010   //��ѹ����
#define SPEED_OVER_ALARM      0X0020   //������ٱ���
#define OVERLOAD_PRO_ALARM    0X0200   //������ر���
//��д����궨��
#define WRITE        06
#define READ         03
//���/������ID�궨��
#define ID_MOTOR_L   01  //��������
#define ID_MOTOR_R   02  //�Ҳ������
//��������Ĵ�����ַ
#define ALARM_CLEAR           0X0026

//�Ĵ�����ʱ�궨��
#define MOTOR_TIMEOUT   0Xffff
//ת�ٱȶ���
#ifdef METEC_MOTOR
#define RATIO_TRANSMISSION      25 
#else
#define RATIO_TRANSMISSION      0.025 
#endif


/******************��������**********************/
extern int16_t VelOfCar, AngOfCar;//����ϵͳ���ٶȺͽ��ٶȼ����ϲ���ƺ��Զ�Ѳ�߹���
/******************��������**********************/


uint8_t bsp_WheelMotor_init(void);
void CarSpeedRead(int16_t* Vel, int16_t *Ang);
void CarSpeedSet(int16_t Vel_Set, int16_t Ang_Set);
void CarMotorFaultCheck(volatile uint16_t * Fault_reg);
/****************************************/











#endif

















