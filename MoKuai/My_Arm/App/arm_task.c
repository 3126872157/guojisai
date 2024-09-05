#include "arm_ctrl.h"
#include "cmsis_os.h"
#include "vofa.h"

#define printf myprintf

extern float set_w;
float targ_pos = 0;
float real_pos = 0;
uint8_t direction = 0; //1ио2об
extern unitree_ctrl_t unitree_Data;

void arm_task(void const * argument)
{
	unitree_pos_pid_ctrl(targ_pos);
	unitree_save_check();
	
	real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
	printf("%f,%f,%f,%f,%f\n", targ_pos, real_pos, unitree_Data.unitree_recv.LW, set_w, unitree_Data.unitree_recv.W);
}
