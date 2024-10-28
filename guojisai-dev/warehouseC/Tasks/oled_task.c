#include "oled_task.h"
#include "oled.h"
#include "FreeRTOS.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "arm_task.h"

uint32_t ms_Timercount1 = 0;
uint16_t time_s = 0;//显示帧数

//显示数据
extern uint8_t my_cali_flag;
extern fp32 my_angle[3];

//按键有关
extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;
extern uint8_t safe_flag;
extern uint8_t arm_safe;
bool_t liucheng_start_flag = 0;

void oled_task(void const * argument)
{
	vTaskDelay(OLED_TASK_INIT_TIME);
	
	while(1)
	{
		ms_Timercount1 ++;
		if (ms_Timercount1 >= 1000)
		{
		  time_s ++; 
		  if(arm_safe == 0 && safe_flag == 0)
			  time_s = 0;
		  ms_Timercount1 = 0;
		}
				
		OLED_ShowString(96, 47, "time:", OLED_6X8);
		OLED_ShowNum(104, 55, time_s, 4, OLED_6X8); 
		
		OLED_ShowString(0, 0, "my_cali_flag = ", OLED_6X8);
		OLED_ShowNum(90, 0, my_cali_flag, 1, OLED_6X8); 
		
		OLED_ShowString(0, 16, "yaw_angle = ", OLED_6X8);
		OLED_ShowSignedNum(0, 30, my_angle[0], 3, OLED_6X8);
		OLED_ShowChar(24, 30 , '.', OLED_6X8);
		OLED_ShowNum(32, 30, (int)(my_angle[0] * 1000) % 1000, 3, OLED_6X8);
		
		OLED_Gram_Refreash();
		
		//按键启动
		if(liucheng_start_flag == 0)
		{
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				arm_safe = 0;
				osDelay(2500);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				safe_flag = 0;
				liucheng_start_flag = 1;
			}
		}
	
		osDelay(1);
	}
}

















