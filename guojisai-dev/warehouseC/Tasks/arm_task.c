#include "arm_task.h"
#include "arm_solver.h"
#include "pid.h"
#include "usbd_cdc_if.h"
#include "unitree_a1.h"

//#define printf myprintf

float targ_pos = 0;
float ramp_targ_pos = 0;
float arm_slow_start_k = 0.0008;
float real_pos = 0;
float arm_zero_pose = 1.14955831f;	//特定零点区间内转到竖直位置的“绝对角度”


uint8_t direction = 0; //1上2下
extern unitree_ctrl_t unitree_Data;
extern serial_servo_t servo_Data;
extern struct arm_solver solver;
extern uint8_t TX_shijue_mode;
extern uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM * 2];

arm_ctrl_point point;

bool_t arm_safe = 1;//机械臂调试保护位
bool_t unitree_init_flag = 0;//大臂上电初始化标志位
bool_t arm_ctrl_signal = 0;

uint16_t servo_start_flag = 99;//每数到100，总线舵机发送信号执行一次(即1s执行一次)

bool_t A1_die_flag = 0;

uint8_t TX_shijue_flag = 99;

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
			ramp_targ_pos = unitree_Data.unitree_recv.Pos;
			osDelay(10);
		}
		
		if(A1_die_flag == 1)
		{
			unitree_torque_ctrl(&unitree_Data, 0);
			unitree_Uart_RE_Init(unitree_rx_buf[0], unitree_rx_buf[1], Unitree_RX_BUF_NUM);
			__HAL_UART_CLEAR_PEFLAG(&UNITREE_MOTOR_HUART);
			A1_die_flag = 0;
		}
		
		//初始转到大臂数值位置
		if(!unitree_init_flag)
		{
			targ_pos = arm_zero_pose;
			if(fabs(unitree_Data.unitree_recv.Pos-targ_pos) <= 0.005f)
			{
				targ_pos = 0;
				real_pos = 0;
				ramp_targ_pos = 0;
				unitree_Data.zero_pose = arm_zero_pose;
				unitree_init_flag = 1;
				arm_ctrl_signal = 1;
			}
		}
		
/**********机械臂解算调试用程序***********/
		else
		{
			servo_start_flag++;
			if(servo_start_flag >= servo_Data.serial_servo_Time)	//100ms发送一次舵机控制信号，查看舵机角度
			{
				if(arm_ctrl_signal)	//不能同时发送
				{
					arm_solve(point.total_angle, point.x, point.y);
					servo_arm_move(solver.a1, solver.a2);	//控制舵机转到制定角度
					targ_pos = -solver.a0;					//设置电机角度
				}
				else
				{
					getServosAngle(3,1,2,3);
				}
				servo_start_flag = 0;
				
				//借用一下地方发送视觉模式
				CDC_Transmit_FS(&TX_shijue_mode,1);
				
			}
/***********************************************/	
		}
		
//		if(TX_shijue_flag >= 100)
//		{
//			CDC_Transmit_FS(&TX_shijue_mode,1);
//			TX_shijue_flag = 0;
//		}
//		TX_shijue_flag ++;
		
		//大臂电机位置控制
		ramp_function(&ramp_targ_pos, targ_pos, arm_slow_start_k); 
		unitree_pos_pid_ctrl(ramp_targ_pos);
		unitree_save_check();
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		osDelay(1);
	}
}
