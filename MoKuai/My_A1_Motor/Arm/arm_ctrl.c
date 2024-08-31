#include "main.h"
//#include "tim.h"
#include "arm_ctrl.h"
//#include "uart_servo.h"
#include "unitree_a1.h"
#include "pid.h"

extern uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];
extern ServoComdDataV3 motor_rx_temp;

unitree_ctrl_t unitree_Data;

pid_type_def unitree_w_pid;
pid_type_def unitree_pos_pid;
const static fp32 unitree_w_pid_K[3] = {UNITREE_W_PID_KP, UNITREE_W_PID_KI, UNITREE_W_PID_KD};
const static fp32 unitree_pos_pid_K[3] = {UNITREE_POS_PID_KP, UNITREE_POS_PID_KI, UNITREE_POS_PID_KD};

float K_tff = 0.01;
float Tf;

//uart_servo_Data_1 uart_servo_Data;
//servo_data_1 servo_data;

//封装力矩控制的函数
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque)
{
	modfiy_torque_cmd(&ctrl->unitree_send, 0, torque);
	UnitreeSend(&ctrl->unitree_send);
	ExtractData(&ctrl->unitree_recv, &motor_rx_temp);
}

//封装混合模式控制
//void unitree_mix_ctrl(unitree_ctrl_t *ctrl, float tff, )

//宇树电机初始化检查零点
void unitree_check_zero_pose(unitree_ctrl_t *ctrl)
{
	unitree_torque_ctrl(ctrl, 0);
	HAL_Delay(10);
	unitree_torque_ctrl(ctrl, 0);
	ctrl->zero_pose = ctrl->unitree_recv.Pos;
}

void unitree_w_pid_ctrl(float w)
{
	float real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
	float tff;
	
	//往正方向转
	if(w >= 0)
	{
		tff = Tf;
		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.W, w));
	}
	if(w < 0)
	{
		tff = -Tf;
		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.W, w));
	}
}

void unitree_pos_pid_ctrl(float pos)
{
	unitree_w_pid_ctrl(PID_calc(&unitree_pos_pid, unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose, pos));
}

void Arm_Init(void)
{
	//幻尔初始化
//	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);
	
	//宇树初始化
	unitree_Usart6_Init(Unitree_rx6_buf[0],Unitree_rx6_buf[1],Unitree_RX_BUF_NUM);
	PID_init(&unitree_w_pid, PID_POSITION, unitree_w_pid_K, 2.0f, 0.2f);	//0.07, 0.0001, 0.1
	PID_init(&unitree_pos_pid, PID_POSITION, unitree_pos_pid_K, 2.0f, 0.2f);
	unitree_check_zero_pose(&unitree_Data);

//	//宇树初始角度
//	uint8_t check = 0;
//	uint8_t a[2] = {0};
//	if(check == 0)
//	{
////		ModifyData(&unitree_Data.unitree_send,0,10,0,0,0,0);
//		UnitreeSend(&unitree_Data.unitree_send);
//		ExtractData(&unitree_Data.unitree_recv,&motor_rx_temp);
//		a[1] = a[0];
//		a[0] = unitree_Data.unitree_recv.Pos;
//		unitree_Data.zero_pose = unitree_Data.unitree_recv.Pos;
//		if(unitree_Data.zero_pose != 0)
//		{
//			if((a[0] - a[1])<0.05||(a[1] - a[0])<0.05)
//			{					
//				unitree_Data.zero_pose = unitree_Data.unitree_recv.Pos;
//				check = 1;
//			}
//		}
//	}
}
