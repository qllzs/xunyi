#include "bsp_trail.h"
/**
  * @brief  ����Ѱ���õ���I/O��
  * @param  ��
  * @retval ��
  */
void TrailGPIOInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*����Ѱ��GPIO�ڵ�ʱ��*/
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
  * @brief  ����Ѱ����IO�ڵ�ƽ״̬��ͬ�Ե���Ľ��ٶȸ��費ͬ��ֵ����ʵ��С�����Զ�Ѳ�߹���     
  *	@param 	 vel -- С�����е��ٶ� ��λ0.01m/s    ang -- ���ٶ� ��λΪ0.01rad/s
  * @retval  1-�ɹ�   0-ʧ��
	* @note �������ż������ɨ�赽����Ϊ0��ɨ�赽����Ϊ1
  */
int8_t  Trail_Scan(int16_t* velocity, int16_t* angular)
{
	//���Ѱ��ɨ�赽��
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























