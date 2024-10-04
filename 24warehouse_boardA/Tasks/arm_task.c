#include "arm_task.h"
#include "arm_solver.h"
#include "pid.h"

//#define printf myprintf

float targ_pos = 0;
float ramp_targ_pos = 0;
float arm_slow_start_k = 0.0008;
float real_pos = 0;
float arm_zero_pose = 1.14955831f;	//特定零点区间内转到竖直位置的“绝对角度”


uint8_t direction = 0; //1上2下
extern unitree_ctrl_t unitree_Data;
extern struct arm_solver solver;

bool_t arm_safe = 1;//机械臂调试保护位
bool_t unitree_init_flag = 0;//大臂上电初始化标志位

float total_angle = 90;
float x = 300;
float y = 300;

uint16_t servo_start_flag = 99;//每数到100，总线舵机发送信号执行一次(即1s执行一次)

uint8_t task_flag;
bool_t claw_flag;

void Task_1(void);
void Task_2(void);
void Task_3(void);
void Task_4(void);

void arm_task(void const * argument)
{
	osDelay(10);
	Arm_Init();
	osDelay(10);
	
	while(1)
	{
		//调试保护
		while(arm_safe)
		{
			unitree_torque_ctrl(&unitree_Data, 0);
			osDelay(10);
		}
		
		//初始转到大臂数值位置
		if(!unitree_init_flag)
		{
			targ_pos = arm_zero_pose;
			if(fabs(unitree_Data.unitree_recv.Pos-targ_pos) <= 0.005f)
			{
				targ_pos = 0;
				real_pos = 0;
				unitree_Data.zero_pose = arm_zero_pose;
				unitree_init_flag = 1;
			}
		}
		
/**********机械臂解算调试用程序***********/
		else
		{
			servo_start_flag++;
			if(servo_start_flag == 500)//500ms发送一次舵机控制信号
			{
				
				
				arm_solve(total_angle, x, y);	//内置舵机控制
				servo_start_flag = 0;
				
				//getServosAngle(3,1,2,3);
			}
			targ_pos = -solver.a0;
/***********************************************/	
		}
		//大臂电机位置控制
		ramp_function(&ramp_targ_pos, targ_pos, arm_slow_start_k); 
		unitree_pos_pid_ctrl(ramp_targ_pos);
		unitree_save_check();
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		
		osDelay(1);
	}
}

void Task_1(void)
{
	x = 280;
	y = -30;
	total_angle = 175;
}

void Task_2(void)
{
	x = 300;
	y = 300;
	total_angle = 90;
}

void Task_3(void)
{
	x = 500;
	y = 350;
	total_angle = 100;
}

void Task_4(void)
{
	x = 300;
	y = 300;
	total_angle = 90;
}
