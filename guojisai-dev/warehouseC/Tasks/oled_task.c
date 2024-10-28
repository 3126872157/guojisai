#include "oled_task.h"
#include "oled.h"
#include "FreeRTOS.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "arm_task.h"
#include "usbd_cdc_if.h"

uint32_t ms_Timercount1 = 0;
uint16_t time_s = 0;//显示帧数

//显示数据
extern uint8_t my_cali_flag;
extern fp32 my_angle[3];
extern shijue_Data shijue_data;//视觉数据帧：球x,y,distance, 二维码x,y,

extern uint8_t IC_data;


void oled_task(void const * argument)
{
	vTaskDelay(OLED_TASK_INIT_TIME);
	
	while(1)
	{
		ms_Timercount1 ++;
		if (ms_Timercount1 >= 50)
		{
		  time_s ++; 
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
		
//		OLED_ShowChar(0, 47 , 'A', OLED_6X8);
//		OLED_ShowString(0, 55, "Red", OLED_6X8);
//		
//		OLED_ShowString(12, 47, "shijue:", OLED_6X8);
//		
//		OLED_ShowNum(56, 47, shijue_data.ball_x, 3, OLED_6X8);
		
		OLED_Gram_Refreash();
		
	
		osDelay(20);
	}
}

















