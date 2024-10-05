#include "flow_task.h"
#include "chassis_behaviour.h"
#include "bodanpan.h"
#include "Bodanpan_Task.h"
#include "arm_control_task.h"

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

//机械臂控制有关参数
extern uint8_t arm_control_mode;


extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;
float shijue_k = -1;


bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

/*******************流程控制模式总览********************
0：机械臂
1：底盘平移加转向位置环
2：底盘配合灰度传感器往前走，检测到白线停
3：阶梯平台底盘配合视觉横向移动，检测到球停
4：机械臂阶梯平台取球
5：无力状态，用于debug控制
********************************************************/

bool_t mode4_task_start = 0;

//控制变量列表，格式如下
//控制模式		para1		para2		para3		到达判断误差值			底盘运动模式
//0				
//1				x			y       	gyro   		distance_tol或gyro_tol	CHASSIS_MOVE_AND_ROTATE
//2				x_speed		y_speed											CHASSIS_V
//3				x_speed		y_speed											CHASSIS_V
//4																			CHASSIS_MOVE_AND_ROTATE
//5

const uint8_t non = 0;

TargetPoints targ_point[] = {
	{1,		 190,	 	-80,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		  	0,			-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 5,	      	0,			0,			non,			CHASSIS_V},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{5,		 non,		non,		non,		non,			CHASSIS_V}
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
		
			if(target.mode == 1)
			{
				//设置底盘运动目标
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
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
				V_mode_x_speed = target.para1;
				
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
				V_mode_y_speed = target.para2;
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < 1)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
					currentTargIndex ++;
				}
			}
			
			if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(mode4_task_start == 0)
				{
					if(fabs(shijue_data.ball_distance)/*距离为最高一层*/)
					{
						arm_control_mode = 1;
						mode4_task_start = 1;
					}	
					
					else if(fabs(shijue_data.ball_distance)/*距离为中间一层*/)
					{
						arm_control_mode = 2;
						mode4_task_start = 1;
					}
					else if(fabs(shijue_data.ball_distance)/*距离为最低一层*/)
					{
						arm_control_mode = 3;
						mode4_task_start = 1;
					}
				}
				if(arm_control_mode == 0)
					currentTargIndex ++;
			}	
			
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(take_a_ball)
					bodanpan_find_ball(ball_x,ball_y);
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
