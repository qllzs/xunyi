#ifndef  __BSP_WHEEL_MOTOR_H__
#define  __BSP_WHEEL_MOTOR_H__
#include "bsp_include.h"


/********************串口宏定义**********************/
// 串口2-USART2
#define  Motor_USARTx                   USART2
#define  Motor_USART_CLK                RCC_APB1Periph_USART2
#define  Motor_USART_APBxClkCmd         RCC_APB1PeriphClockCmd
#define  Motor_USART_BAUDRATE           38400

// USART GPIO 引脚宏定义
#define  Motor_USART_GPIO_CLK           (RCC_APB2Periph_GPIOA)
#define  Motor_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  Motor_USART_TX_GPIO_PORT       GPIOA   
#define  Motor_USART_TX_GPIO_PIN        GPIO_Pin_2
#define  Motor_USART_RX_GPIO_PORT       GPIOA
#define  Motor_USART_RX_GPIO_PIN        GPIO_Pin_3

#define  Motor_USART_IRQ                USART2_IRQn
#define  Motor_USART_IRQHandler         USART2_IRQHandler
//DMA接收通道参数宏定义
#define DMA_Rx_BUFFER_SIZE              16
#define Motor_USARTx_DMA_Rx_Channel     DMA1_Channel6
#define RCC_AHBPeriph_DMA_USARTx_Motor   RCC_AHBPeriph_DMA1
//DMA发送通道参数宏定义
#define DMA_Tx_BUFFER_SIZE              8
#define Motor_USARTx_DMA_Tx_Channel     DMA1_Channel7
#define Motor_USARTx_DMA_FLAG_TC        DMA1_FLAG_TC7
/********************驱动器宏定义**********************/
//功能码宏定义
#define W_S_REG        0x06 //写单个寄存器
#define R_REG          0x03 //读寄存器


//读写寄存器地址宏定义
#define CON_MOD_REG        0X0011   //控制模式0：内部速度模式  1：电位器模式
#define INTER_SPEED_REG    0X0012   //RPM
#define KP_SPEED_REG       0X0013   //速度pid增益10-10000
#define KI_SPEED_REG       0X0014   //速度pid积分10-10000
#define STOP_MODE_REG      0X0015   //停机方式 0：自由停机 1：刹车停机   2：减速停机
#define BREAK_STRENTH_REG  0X0016   //刹车停机强度%
#define ACCELERATE_REG     0X0017   //加速度rpm/ms
#define DECELERATE_REG     0X0018   //减速度rpm/ms
#define GAIN_REG           0X0019   //电位器调速增益rpm/v
#define OVER_CURRENT_REG   0X001a   //过载电流值 A
#define OVER_TIME_REG      0X001b   //过载时间ms
#define ALARM_CURRENT_REG  0X001c   //过流报警阈值A
#define PWM_FREQUENCY_REG  0X001d   //PWM开关频率4-16kHz
#define LIMIT_DUTY_REG     0X001e   //PWM限制最大占空比10%-100%
#define MOTOR_POLE_REG     0X001f   //电机的极对数
#define X0_FUCTION_REG     0X0020   //复用X0引脚的功能 0：X0配置为刹车功能  1：X0配置为多段速功能
#define INTER_SPEED2_REG   0X0021   //第二内部速度RPM
#define BAUDRATE_REG       0X0022   //通讯波特率 0:9600  1:38400   2:57600   3:115200
#define DRIVER_NUM_REG     0X0023   //驱动器编号1-255
#define IO_TOGGLE_REG      0X0024   //输入I/O逻辑取反1:001（run取反） 2:010（DIR取反） 4:100（break取反）
#define PARA_SAVE          0X0025   //参数保存寄存器，写1保存
//只读16位寄存器地址
#define CURRENT_SPEED_REG      0X0001   //RPM
#define HAL_STAT_REG           0X0002   //5-1-3-2-6-4
#define FAULT_STAT_REG         0X0003   //参考故障代码
#define BUS_VOL_REG            0X0004   //V
#define CURRENT_REG            0X0005   //001A
#define DRIVER_TEMP_REG        0X0006   //℃

//故障代码宏定义
#define LOCKED_ALARM          0X0001   //电机堵转报警
#define OVER_CURRENT_ALARM    0X0002   //过流报警
#define HAL_FAULT_ALARM       0X0004   //霍尔故障报警
#define VOL_DOWM_ALARM        0X0008   //欠压报警
#define VOL_UP_ALARM          0X0010   //过压报警
#define SPEED_OVER_ALARM      0X0020   //电机过速报警
#define OVERLOAD_PRO_ALARM    0X0200   //电机过载保护
//读写命令宏定义
#define WRITE        06
#define READ         03
//电机/驱动器ID宏定义
#define ID_MOTOR_L   01  //左侧电机编号
#define ID_MOTOR_R   02  //右侧电机编号
//报警清除寄存器地址
#define ALARM_CLEAR           0X0026

//寄存器超时宏定义
#define MOTOR_TIMEOUT   0Xffff
//转速比定义
#ifdef METEC_MOTOR
#define RATIO_TRANSMISSION      25 
#else
#define RATIO_TRANSMISSION      0.025 
#endif


/******************变量声明**********************/
extern int16_t VelOfCar, AngOfCar;//整个系统的速度和角速度兼容上层控制盒自动巡线功能
/******************函数声明**********************/


uint8_t bsp_WheelMotor_init(void);
void CarSpeedRead(int16_t* Vel, int16_t *Ang);
void CarSpeedSet(int16_t Vel_Set, int16_t Ang_Set);
void CarMotorFaultCheck(volatile uint16_t * Fault_reg);
/****************************************/











#endif

















