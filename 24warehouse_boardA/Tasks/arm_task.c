#include "arm_task.h"
#include "arm_solver.h"

//#define printf myprintf

extern float set_w;
float targ_pos = 0;
float real_pos = 0;

uint8_t direction = 0; //1ио2об
extern unitree_ctrl_t unitree_Data;
extern struct arm_solver solver;

float a = 90;
float b = 300;
float c = 300;
uint8_t servo_flag = 99;


void arm_task(void const * argument)
{
	osDelay(10);
	Arm_Init();
	osDelay(10);
	osDelay(2000);
	while(1)
	{
		servo_flag++;
		if(servo_flag == 100)
		{
			arm_ctrl(a, b, c);
			servo_flag = 0;
		}

		unitree_pos_pid_ctrl(-solver.a0);
		
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		//printf("%f,%f,%f,%f,%f\n", targ_pos, real_pos, unitree_Data.unitree_recv.LW, set_w, unitree_Data.unitree_recv.W);
		unitree_save_check();
		osDelay(10);
//		getServosAngle(3,1,2,3);
		
	}
}
