#include "Bodanpan_Task.h"


pid_type_def bodanpan_angle_pid,
             bodanpan_real_angle_pid,
			 bodanpan_speed_pid;

uint8_t ball_num=0;//储蛋机构内已有球的个数
bool_t a_new_ball_in = 0;
extern uint8_t IC_data; //Uart_Task中接收到的，当前位置扭蛋球的IC卡数据
uint8_t err_IC_data = 0xFF;//取球错误标识
extern Pan_t bodanpan;
extern motor_measure_t motor_chassis[5];

void IC_story(uint8_t data);

void Bodanpan_Task(void const * argument)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	bodanpan_motor_init();
	while(1)
	{
		if(a_new_ball_in)
		{
			bodanpan_position_set(1,1);
			osDelay(1500);
			//如果检测到的还是上一个球的数据或者没有检测到,来回动一下，防止IC卡读不到
			if(IC_data == bodanpan.IC_date_pan[ball_num-1] || IC_data == 0)
			{
				bodanpan_position_set(-1,1);
				osDelay(500);
				bodanpan_position_set(1,1);
				osDelay(1000);
				if(IC_data != bodanpan.IC_date_pan[ball_num-1] && IC_data != 0)
					IC_story(IC_data);
				else
					IC_story(err_IC_data);
			}
			else IC_story(IC_data);
			//如果还是没有数据，则默认夹球失误，给当前位置存入错误标识err_IC_data
			
			a_new_ball_in = 0;
		}

		osDelay(5);
	}
}
void bodanpan_motor_init(void)
{
	bodanpan_init();
	
	fp32 motor_angle_pan_pid[3] = {BODANPAN_ANGLE_KP, BODANPAN_ANGLE_KI, BODANPAN_ANGLE_KD};
	PID_init(&bodanpan_angle_pid, PID_POSITION, motor_angle_pan_pid, M2006_MOTOR_ANGLE_PAN_PID_MAX_OUT, M2006_MOTOR_ANGLE_PAN_PID_MAX_IOUT);
		
	fp32 motor_speed_pan_pid[3] = {BODANPAN_SPEED_KP, BODANPAN_SPEED_KI, BODANPAN_SPEED_KD};
	PID_init(&bodanpan_speed_pid, PID_POSITION, motor_speed_pan_pid, M2006_MOTOR_SPEED_PAN_PID_MAX_OUT, M2006_MOTOR_SPEED_PAN_PID_MAX_IOUT);
}

void bodanpan_motor_control(void)
{
	bodanpan.code_now = motor_chassis[4].code;
	
	PID_calc(&bodanpan_angle_pid,bodanpan.code_now,bodanpan.code_set);
	
	bodanpan.speed_set = bodanpan_angle_pid.out/10.0f;
	bodanpan.speed=motor_chassis[4].speed_rpm/10.0f;
	
	PID_calc(&bodanpan_speed_pid,bodanpan.speed,bodanpan.speed_set);
	
	motor_chassis[4].given_current=bodanpan_speed_pid.out;
	
	CAN_cmd_pan(motor_chassis[4].given_current);

}

void IC_story(uint8_t data)
{
	bodanpan.IC_date_pan[ball_num] = data;
	bodanpan.box_state[ball_num] = 1;
	ball_num++;

}

