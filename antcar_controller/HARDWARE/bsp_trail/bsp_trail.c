#include "bsp_trail.h"
/**
  * @brief  配置寻迹用到的I/O口
  * @param  无
  * @retval 无
  */
void TrailGPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*开启寻迹GPIO口的时钟*/
	RCC_APB2PeriphClockCmd(Trail_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = Trail_1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Trail_2_PIN;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Trail_3_PIN;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Trail_4_PIN;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Trail_5_PIN;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = Trail_6_PIN;
	GPIO_Init(Trail_GPIO_PORT, &GPIO_InitStructure);
}
/**
  * @brief  根据寻迹的IO口电平状态不同对电机的角速度赋予不同的值，以实现小车的自动巡线功能     
  *	@param 	 vel -- 小车运行的速度 单位0.01m/s    ang -- 角速度 单位为0.01rad/s
  * @retval  1-成功   0-失败
	* @note 红外引脚检测结果：扫描到黑线为0，扫描到白线为1
  */
int8_t  Trail_Scan(int16_t* velocity, int16_t* angular)
{
	//检查寻迹扫描到的
	uint16_t TrailPortVal = TRAIL_WHITE_Port_VALE;
	TrailPortVal = GPIO_ReadInputData(Trail_GPIO_PORT);
	TrailPortVal = TrailPortVal & TRAILPIN;
	TrailPortVal = TrailPortVal >> 6;
	if(TrailPortVal == TRAIL_WHITE_Port_VALE)
		return 0;
	switch(TrailPortVal)
	{
			case PARTIAL_NONE:
			{
				*velocity = SPEED_TRAIL;
				*angular = 0;
				break;
			}
			case PARTIAL_LEFT_1:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_RIGHT_1;
				break;
			}
			case PARTIAL_LEFT_2:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_RIGHT_2;
				break;
			}
			case PARTIAL_LEFT_3:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_RIGHT_3;
				break;
			}
			case PARTIAL_RIGT_1:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_LEFT_1;
				break;
			}
			case PARTIAL_RIGT_2:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_LEFT_2;
				break;
			}
			case PARTIAL_RIGT_3:
			{
				*velocity = SPEED_TRAIL;
				*angular = TURN_ANGULAR_LEFT_3;
				break;
			}
			case NONEDETECT:
			{
				*velocity = 0;
				*angular = 0;
				break;
			}			
	}
	return 1;
}























