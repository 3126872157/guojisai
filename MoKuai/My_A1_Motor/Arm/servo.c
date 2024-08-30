#include "Arm_Struct.h"
#include "tim.h"

void servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

void servo_Move(uint8_t pos1,uint8_t pos2,uint8_t pos3,uint8_t pos4)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,pos1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,pos2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,pos3);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,pos4);
}
