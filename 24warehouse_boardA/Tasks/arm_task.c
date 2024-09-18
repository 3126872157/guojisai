#include "arm_task.h"
#include "arm_solver.h"

//#define printf myprintf

extern float set_w;
float targ_pos = 0;
float real_pos = 0;

uint8_t direction = 0; //1��2��
extern unitree_ctrl_t unitree_Data;
extern struct arm_solver solver;


bool_t arm_safe = 1;//��е�۵��Ա���λ
float total_angle = 90;
float x = 300;
float y = 300;
uint8_t servo_start_flag = 99;//ÿ����100�����߶�������ź�ִ��һ��(��1sִ��һ��)


void arm_task(void const * argument)
{
	osDelay(10);
	Arm_Init();
	osDelay(10);
	while(1)
	{
		while(arm_safe);
		
/*		*********��е�۽�������ó���***********/
		servo_start_flag++;
		if(servo_start_flag == 100)
		{
			arm_ctrl(total_angle, x, y);
			servo_start_flag = 0;
		}
		unitree_pos_pid_ctrl(-solver.a0);
/***********************************************/	
		
//		unitree_pos_pid_ctrl(targ_pos);
		
		
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		//printf("%f,%f,%f,%f,%f\n", targ_pos, real_pos, unitree_Data.unitree_recv.LW, set_w, unitree_Data.unitree_recv.W);
		unitree_save_check();
		osDelay(10);
//		getServosAngle(3,1,2,3);
		
	}
}
