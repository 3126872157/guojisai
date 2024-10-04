#include "arm_task.h"
#include "arm_solver.h"
#include "pid.h"

//#define printf myprintf

float targ_pos = 0;
float ramp_targ_pos = 0;
float arm_slow_start_k = 0.0008;
float real_pos = 0;
float arm_zero_pose = 1.14955831f;	//�ض����������ת����ֱλ�õġ����ԽǶȡ�


uint8_t direction = 0; //1��2��
extern unitree_ctrl_t unitree_Data;
extern struct arm_solver solver;

bool_t arm_safe = 1;//��е�۵��Ա���λ
bool_t unitree_init_flag = 0;//����ϵ��ʼ����־λ

float total_angle = 90;
float x = 300;
float y = 300;

uint16_t servo_start_flag = 99;//ÿ����100�����߶�������ź�ִ��һ��(��1sִ��һ��)

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
		//���Ա���
		while(arm_safe)
		{
			unitree_torque_ctrl(&unitree_Data, 0);
			osDelay(10);
		}
		
		//��ʼת�������ֵλ��
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
		
/**********��е�۽�������ó���***********/
		else
		{
			servo_start_flag++;
			if(servo_start_flag == 500)//500ms����һ�ζ�������ź�
			{
				
				
				arm_solve(total_angle, x, y);	//���ö������
				servo_start_flag = 0;
				
				//getServosAngle(3,1,2,3);
			}
			targ_pos = -solver.a0;
/***********************************************/	
		}
		//��۵��λ�ÿ���
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
