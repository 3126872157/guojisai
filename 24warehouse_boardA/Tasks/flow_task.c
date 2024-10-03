#include "flow_task.h"
#include "chassis_behaviour.h"

#define tol 2

//控制底盘运动
extern chassis_move_t chassis_move;
extern chassis_mode_e chassis_behaviour_mode;
extern bool_t chassis_code_reset_flag;

//CHASSIS_V模式下三个方向给定速度
extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

extern shijue_Data shijue_data;
float shijue_k = -1;

//控制变量列表，格式如下
//底盘或者机械臂   x方向走的距离   y方向走的距离   到达判断误差值   底盘运动模式
//1为底盘0为机械臂    单位cm         每次清零          单位cm       3是走和转
TargetPoints targ_point[] = {
	{1,		 80,	 0,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		80,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		-80,	 0,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		-80,	tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 80,	0,	    tol,		CHASSIS_V},
	{2,		 0,		80,		tol,		CHASSIS_V},
	{2,		-80,	 0,		tol,		CHASSIS_V},
	{2,		 0,		-80,	tol,		CHASSIS_V},
	{3,		 0,		0,		tol,		CHASSIS_V}
};

//当前所在位置序号
uint8_t currentTargIndex = 0;
//完成任务标志
uint8_t isFinished = 0;

void flow_task(void const * argument)
{
	while(1)
	{
		//判断所有步骤是否走完
		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
		{
			//读出当前目标
			TargetPoints target = targ_point[currentTargIndex];
			
			//目前动底盘
			if(target.mode == 1)
			{
				//设置底盘运动目标
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.x;
				chassis_move.y_set = target.y;
				
				//判断误差
				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
				if(distance < target.tolerance)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;

				if(target.x > 0) V_mode_x_speed = 5;
				else if(target.x < 0) V_mode_x_speed = -5;
				else V_mode_x_speed = 0;
					
				if(target.y > 0) V_mode_y_speed = 5;
				else if(target.y < 0) V_mode_y_speed = -5;
				else V_mode_y_speed = 0;

				
//				chassis_move.x_set = 0;
//				chassis_move.y_set = shijue_data.ball_x/shijue_k;	
				
				if(fabs(chassis_move.x-target.x) < target.tolerance)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex++;
				}
				
			}
			else if(target.mode == 3)//速度调试模式
			{
				chassis_behaviour_mode = target.chassis_mode;
			}
		}
		
		else
		{
			isFinished ++;
			//删除当前任务
			vTaskDelete(NULL);
		}
		osDelay(1);
	}
}
