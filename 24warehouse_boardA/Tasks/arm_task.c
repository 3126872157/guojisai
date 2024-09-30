#include "arm_task.h"
#include "arm_solver.h"

//#define printf myprintf

extern float set_w;
float targ_pos = 0;
float real_pos = 0;
float my_zero_pose = -0.23;

uint8_t direction = 0; //1上2下
extern unitree_ctrl_t unitree_Data;
extern struct arm_solver solver;
extern uint16_t claw_pos;
uint16_t huadao_pwm = 600;
uint16_t tulun_pwm = 250;

bool_t arm_safe = 1;//机械臂调试保护位
bool_t unitree_init_flag = 0;//大臂上电初始化标志位

float total_angle = 175;
float x = 300;
float y = -40;

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
		while(arm_safe);//调试保护位
		if(!unitree_init_flag)
		{
			targ_pos = my_zero_pose - unitree_Data.zero_pose;
			if(fabs(real_pos-targ_pos) <= 0.005f)
			{
				targ_pos = 0;
				real_pos = 0;
				unitree_Data.zero_pose = my_zero_pose;
				unitree_init_flag = 1;
			}
		}
		else
		{
			
/*		*********机械臂解算调试用程序***********/
			servo_start_flag++;
			if(servo_start_flag == 100)//1s发送一次舵机控制信号
			{
				//夹爪控制已包含在内，通过给claw_pos赋值实现
				
				if(task_flag==1) Task_1();
				else if(task_flag==2) Task_2();
				else if(task_flag==3) Task_3();
				else if(task_flag==4) Task_4();
				
				if(claw_flag==1) claw_pos = 410;
				else if(claw_flag==0) claw_pos = 500;
				
				arm_ctrl(total_angle, x, y);
				servo_start_flag = 0;
			}
			targ_pos = -solver.a0;
/***********************************************/	
		}
		
		unitree_pos_pid_ctrl(targ_pos);
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		unitree_save_check();
		huadao_control(huadao_pwm);
		tulun_control(tulun_pwm);
		osDelay(10);
//		getServosAngle(3,1,2,3);
		
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
