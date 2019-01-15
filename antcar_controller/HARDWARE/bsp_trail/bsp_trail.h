#ifndef __BSP_TRAIL_H__
#define __BSP_TRAIL_H__
#include "bsp_include.h"
/*             引脚宏定义                   */
#define Trail_GPIO_PORT              GPIOD                      
#define Trail_GPIO_CLK               RCC_APB2Periph_GPIOD

#define Trail_1_PIN                  GPIO_Pin_15                 

#define Trail_2_PIN                  GPIO_Pin_14                                      

#define Trail_3_PIN                  GPIO_Pin_13                 

#define Trail_4_PIN                  GPIO_Pin_12                 

#define Trail_5_PIN                  GPIO_Pin_11                 

#define Trail_6_PIN                  GPIO_Pin_10 


/*             寻迹检测宏定义                   */
#if 0 //四个寻迹模块
#define TRAILPIN	                   0x3c0
#define PARTIAL_NONE                 0X6                    
#define PARTIAL_LEFT_1               0X2 
#define PARTIAL_LEFT_2               0X3
#define PARTIAL_LEFT_3               0X1
#define PARTIAL_RIGT_1               0X4 
#define PARTIAL_RIGT_2               0XC
#define PARTIAL_RIGT_3               0X8
#define NONEDETECT                   0X0

#define TURN_ANGULAR_LEFT_1          10       
#define TURN_ANGULAR_LEFT_2          20
#define TURN_ANGULAR_LEFT_3          30
#define TURN_ANGULAR_RIGHT_1         -10 
#define TURN_ANGULAR_RIGHT_2         -20
#define TURN_ANGULAR_RIGHT_3         -30

#else   //六个寻迹模块

#define TRAILPIN                     0xfc0
//偏航定义-待定
#define PARTIAL_NONE                 0X33                    
#define PARTIAL_LEFT_1               0X3B 
#define PARTIAL_LEFT_2               0X39
#define PARTIAL_LEFT_3               0X3D
#define PARTIAL_LEFT_4               0X3C
#define PARTIAL_LEFT_5               0X3E
#define PARTIAL_RIGT_1               0X37 
#define PARTIAL_RIGT_2               0X47
#define PARTIAL_RIGT_3               0X4f
#define PARTIAL_RIGT_4               0X0f
#define PARTIAL_RIGT_5               0X1f

#define TURN_ANGULAR_LEFT_1          10       
#define TURN_ANGULAR_LEFT_2          20
#define TURN_ANGULAR_LEFT_3          30
#define TURN_ANGULAR_LEFT_4          20
#define TURN_ANGULAR_LEFT_5          30

#define TURN_ANGULAR_RIGHT_1         -10 
#define TURN_ANGULAR_RIGHT_2         -20
#define TURN_ANGULAR_RIGHT_3         -30
#define TURN_ANGULAR_RIGHT_4         -20
#define TURN_ANGULAR_RIGHT_5         -30
#define NONEDETECT                   0X0
#endif

//寻迹速度定义
#define SPEED_TRAIL                       1 
#define TRAIL_BLACK_Pin_VALE              1
#define TRAIL_WHITE_Pin_VALE              0

#define TRAIL_BLACK_Port_VALE             0x3f
#define TRAIL_WHITE_Port_VALE             0x0





/*                      外部函数的声明             */





void TrailGPIOInit(void);
int8_t Trail_Scan(int16_t* velocity, int16_t* angular);
#endif  //__BSP_TRAIL_H__
