#include "arm_ctrl.h"
#include "arm_solver.h"
//--------------------------------宇树电机变量--------------------------------
// 串口缓冲
extern uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM];
extern ServoComdDataV3 motor_rx_temp;
// 电机控制结构体
unitree_ctrl_t unitree_Data;
// pid相关
pid_type_def unitree_w_pid;
pid_type_def unitree_pos_pid;
const static fp32 unitree_w_pid_K[3] = {UNITREE_W_PID_KP, UNITREE_W_PID_KI, UNITREE_W_PID_KD};
const static fp32 unitree_pos_pid_K[3] = {UNITREE_POS_PID_KP, UNITREE_POS_PID_KI, UNITREE_POS_PID_KD};
// 控制相关
float K_tff = 0.01; // 前馈力矩因子，想着是前馈力矩跟角速度有关
float K_set_w = 5;
float Tf = 0.08;	//分情况讨论前馈力矩0.08,0.145
float Tf_up = 1.0;//0.1
float Tf_down = 0;
float set_w = 0;
float kw = 4.0;
float up_w = 1.2;
float down_w = -0.2;
uint8_t dangerous_count = 0;	//超出最大力矩次数

//--------------------------------总线舵机变量--------------------------------
serial_servo_t servo_Data;
extern uint8_t Servo_Rx_Data[20];	//生的数据
//说明：1号舵机为第二关节，增大为往上扬
//	    2号舵机为第三关节，增大为往下按
//		3号舵机为夹爪，增大为张开
//		4号舵机为拨蛋板，增大为归位
uint16_t claw_pos = 470;
uint16_t claw_catch_pos = 410;
uint16_t claw_loose_pos = 510;
uint16_t claw_middle_pos = 470;
uint16_t paidanban_pos = 500;
uint16_t bogan_zhunbei_pos = 850;
uint16_t bogan_jiqiu_pos = 650;
uint16_t bogan_shouqi_pos = 250;


//--------------------------------滑道、凸轮舵机变量----------------------------
uint16_t huadao_vertical_pwm = 800;//900垂直600放球(范围250-1250)
uint16_t huadao_slope_out_pwm = 770;
uint16_t huadao_slope_in_pwm = 1100;
uint16_t tulun_up_pwm = 1250;//1250升起250落下(范围250-1250)
uint16_t tulun_down_pwm = 250;

//--------------------------------机械臂与解算变量----------------------------
// 逆运动学解算结构体
struct arm_solver solver;
uint8_t error;

//--------------------------------宇树电机函数--------------------------------
// 封装力矩控制的函数
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque)
{
	modfiy_torque_cmd(&ctrl->unitree_send, 0, torque);
	UnitreeSend(&ctrl->unitree_send);
	ExtractData(&ctrl->unitree_recv, &motor_rx_temp);
}

// 封装速度控制的函数
void unitree_speed_ctrl(unitree_ctrl_t *ctrl, float speed, float kw)
{
	modfiy_speed_cmd(&ctrl->unitree_send, 0, speed, kw);
	UnitreeSend(&ctrl->unitree_send);
	ExtractData(&ctrl->unitree_recv, &motor_rx_temp);
}

//宇树运行安全卫士：检测力矩有无过大
void unitree_save_check(void)
{
	if(unitree_Data.unitree_recv.T >= UNITREE_MAX_TORQUE || unitree_Data.unitree_recv.T <= -UNITREE_MAX_TORQUE)
	{
		//当前力矩过大，设置力矩为零
		unitree_torque_ctrl(&unitree_Data, 0);
		dangerous_count ++;
	}
}


// 速度pid控制
void unitree_w_pid_ctrl(float w)
{
	float real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;

	// 往正方向转
	if (unitree_pos_pid.error[0] > 0.01f)	//在0.01误差外
	{
//		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.LW, w));
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, Tf_up, 0, w, 0, kw);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
	}
	if (unitree_pos_pid.error[0] < -0.01f)
	{
//		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.LW, w));
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, Tf_down, 0, w, 0, kw);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
	}
	else
	{
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, 0, w, 0, kw);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
	}
}

// 位置pid控制
void unitree_pos_pid_ctrl(float pos)
{
	if(pos > UNITREE_MAX_POS || pos < -UNITREE_MAX_POS)
		return;
	
	set_w = PID_calc(&unitree_pos_pid, unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose, pos);
	unitree_w_pid_ctrl(set_w);
}

//特化的宇树A1位置控制
void unitree_move(uint8_t flag, float pos, float w)
{
	if (flag == 1) // 向上
	{
		// 排除向下的情况
		if ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) > pos)
			return;
		// 大位移用速度模式
		while ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) < (pos - 0.08f))
		{
			// 加上正的前馈力矩，w为1.2，kw为3
			modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, 0, up_w, 0, 3);
			UnitreeSend(&unitree_Data.unitree_send);
			ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
			osDelay(1);
		}
		// 小位移用位置模式，kp = 0.02 , kw = 3
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, (unitree_Data.zero_pose + pos), 0, 0.006, 3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
		osDelay(1);
	}
	if (flag == 2) // 向下
	{
		if ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) < pos)
			return;
		while ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) > (pos + 0.08f))
		{
			modfiy_mix_cmd(&unitree_Data.unitree_send, 0, -Tf, 0, down_w, 0, 3);
			UnitreeSend(&unitree_Data.unitree_send);
			ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
			osDelay(1);
		}
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, -Tf, (unitree_Data.zero_pose + pos), 0, 0.006, 3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
		osDelay(1);
	}
}


//----------------------------------舵机函数--------------------------------
//移动关节上的总线舵机
void servo_arm_move(float angle1, float angle2)
{
	float input1;
	float input2;
	
	input1 = 500.0f - angle1 / 0.24f;
	input2 = 500.0f + angle2 / 0.24f;
	
	moveServos(4, servo_Data.serial_servo_Time, 1, (uint16_t)input1, 2, (uint16_t)input2, 3, claw_pos, 4, paidanban_pos);
}

//机械爪夹取，0夹取，1松开，2中间位置
void claw_control(uint8_t mode)
{
	switch(mode)
	{
		case 0:
			claw_pos = claw_catch_pos;
			break;
		case 1:
			claw_pos = claw_loose_pos;
			break;
		case 2:
			claw_pos = claw_middle_pos;
			break;
	}
}


//滑道斜率调整，2为向内扣，1为放球，0为垂直
void huadao_control(bool_t is_put_ball)
{
	if(is_put_ball == 1)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_slope_out_pwm);//700放球(范围250-1250)
	}
	else if(is_put_ball == 0)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_vertical_pwm);//800垂直
	}
	else
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_slope_in_pwm);//1200
	}
}	
//顶球，凸轮调整
void tulun_control(bool_t is_up)
{
	if(is_up)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, tulun_up_pwm);//1250升起250落下
	}
	else
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, tulun_down_pwm);
	}
	
}


//圆盘机拨球控制
void bogan_control(uint8_t mode)
{
	if(mode == 0)	//收起
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_shouqi_pos);//1250
	}
	else if(mode == 1)	//击球
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_jiqiu_pos);//800
	}
	else if(mode == 2)	//准备击球
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_zhunbei_pos);//250
	}
	
}


//需修改
//--------------------------------总机械臂函数--------------------------------
void arm_solve(float end_angle, float x, float y)
{
	error = arm_solver_analyze(&solver, end_angle, x, y);
	if(!error)
	{
		// 输入a0为弧度制		
//		servo_arm_move(solver.a1, solver.a2);
	}
	else
	{
		
	}
}

// 机械臂初始化函数
void Arm_Init(void)
{
	// 逆运动学解算初始化
	arm_solver_init(&solver, 140.0f, 24.38f, 358.13f, 150.0f, 188.17f);
	
	// 幻尔舵机初始化
	serial_servo_UART_Init();
	servo_Data.serial_servo_Time = 400;
	
	// 宇树关节电机初始化
	unitree_Uart_Init(unitree_rx_buf[0], unitree_rx_buf[1], Unitree_RX_BUF_NUM);
	PID_init(&unitree_w_pid,   PID_POSITION, unitree_w_pid_K,   UNITREE_W_PID_MAX_OUT,   UNITREE_W_PID_MAX_IOUT);
	PID_init(&unitree_pos_pid, PID_POSITION, unitree_pos_pid_K, UNITREE_POS_PID_MAX_OUT, UNITREE_POS_PID_MAX_IOUT);
}
