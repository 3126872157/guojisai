#include "Bodanpan_Task.h"


pid_type_def bodanpan_angle_pid,
             bodanpan_real_angle_pid,
			 bodanpan_speed_pid;

uint8_t ball_num=0;//储蛋机构内已有球的个数
bool_t a_new_ball_in_flag = 0;
//extern uint8_t IC_byte;
uint8_t IC_data_RX; //Uart_Task中接收到的，当前位置扭蛋球的IC卡数据

extern Pan_t bodanpan;
extern motor_measure_t motor_chassis[5];

void RFID_STORY(void);

void Bodanpan_Task(void const * argument)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	bodanpan_motor_init();
	while(1)
	{
		if(a_new_ball_in_flag)
		{
			bodanpan_position_set(1,1);
			osDelay(50);
			if(IC_data_RX==bodanpan.IC_date[ball_num-1]||IC_data_RX==0)
				osDelay(1000);
			if(IC_data_RX==bodanpan.IC_date[ball_num-1]||IC_data_RX==0)
				osDelay(1000);
			RFID_STORY();
			a_new_ball_in_flag = 0;
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
	PID_init(&bodanpan_speed_pid, PID_DELTA, motor_speed_pan_pid, M2006_MOTOR_SPEED_PAN_PID_MAX_OUT, M2006_MOTOR_SPEED_PAN_PID_MAX_IOUT);
}

void bodanpan_motor_control(void)
{
	bodanpan.code_now = motor_chassis[4].code;
	
	PID_calc(&bodanpan_angle_pid,bodanpan.code_now,bodanpan.code_set);
	
	bodanpan.speed_set = bodanpan_angle_pid.out/10.0f;
	bodanpan.speed=motor_chassis[4].speed_rpm/10.0f;
	
	PID_calc(&bodanpan_speed_pid,bodanpan.speed,bodanpan.speed_set);
	
	motor_chassis[4].given_current=bodanpan_speed_pid.out;
	
	if(fabs(bodanpan.code_set-bodanpan.code_now)>2000)
		CAN_cmd_pan(motor_chassis[4].given_current);
	else
	{
		bodanpan.speed_set = 0;
		CAN_cmd_pan(0);
		bodanpan_angle_pid.Iout = 0;
	}
		
		
}

void RFID_STORY(void)
{
	if(ball_num==0||IC_data_RX!=bodanpan.IC_date[ball_num-1])
	{
			bodanpan.IC_date[ball_num]=IC_data_RX;
			bodanpan.box_state[ball_num]=1;
			ball_num++;
	}
}

