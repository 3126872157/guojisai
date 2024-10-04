#include "flow_task.h"
#include "chassis_behaviour.h"

#define distance_tol 2
#define gyro_tol 0.2f
//控制底盘运动
extern chassis_move_t chassis_move;
extern chassis_mode_e chassis_behaviour_mode;
extern bool_t chassis_code_reset_flag;

//CHASSIS_V模式下三个方向给定速度
extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

//灰度传感器数据，0为白色红色，1为黑色，目前受硬件限制只有一路数据gray_data[1]
extern uint8_t gray_data[2];


extern shijue_Data shijue_data;
float shijue_k = -1;

//控制变量列表，格式如下
//底盘或者机械臂   x方向走的距离   y方向走的距离   转向的角度  到达判断误差值   底盘运动模式
//1为底盘			单位cm        	 每次清零      单位cm      				 3是走和转
//0为机械臂
//2为底盘配合灰度传感器往前走，检测到白线停
//3为底盘配合视觉横向移动，检测到球停
//4为无力状态，用于手动控制
TargetPoints targ_point[] = {
	{1,		 190,	 -80,	0,		distance_tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		  0,	-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 0,	      0,	0,		distance_tol,		CHASSIS_V},
	{3,		 0,		  0,	0,		distance_tol,		CHASSIS_V},
	{4,		 0,	     40,	0,		gyro_tol,			CHASSIS_V},
	{1,		-80,	 0,		0,		distance_tol,		CHASSIS_V},
	{1,		 0,		-80,	0,		distance_tol,		CHASSIS_V},
	{3,		 0,		0,		0,		distance_tol,		CHASSIS_V}
};

//当前所在位置序号
uint8_t currentTargIndex = 0;
//完成任务标志
uint8_t isFinished = 0;

bool_t flag;

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
				chassis_move.gyro_set = target.gyro;
				//判断误差
				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.gyro) < gyro_tol)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_x_speed = 5;
				
				if(gray_data[1] == 0)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 3)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_y_speed = 5;
				
				if(flag/*视觉看到*/)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 4)
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
