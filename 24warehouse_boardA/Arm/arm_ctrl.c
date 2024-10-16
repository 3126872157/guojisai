#include "arm_ctrl.h"
#include "arm_solver.h"
//--------------------------------�����������--------------------------------
// ���ڻ���
extern uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM];
extern ServoComdDataV3 motor_rx_temp;
// ������ƽṹ��
unitree_ctrl_t unitree_Data;
// pid���
pid_type_def unitree_w_pid;
pid_type_def unitree_pos_pid;
const static fp32 unitree_w_pid_K[3] = {UNITREE_W_PID_KP, UNITREE_W_PID_KI, UNITREE_W_PID_KD};
const static fp32 unitree_pos_pid_K[3] = {UNITREE_POS_PID_KP, UNITREE_POS_PID_KI, UNITREE_POS_PID_KD};
// �������
float K_tff = 0.01; // ǰ���������ӣ�������ǰ�����ظ����ٶ��й�
float K_set_w = 5;
float Tf = 0.08;	//���������ǰ������0.08,0.145
float Tf_up = 1.0;//0.1
float Tf_down = 0;
float set_w = 0;
float kw = 4.0;
float up_w = 1.2;
float down_w = -0.2;
uint8_t dangerous_count = 0;	//����������ش���

//--------------------------------���߶������--------------------------------
serial_servo_t servo_Data;
extern uint8_t Servo_Rx_Data[20];	//��������
//˵����1�Ŷ��Ϊ�ڶ��ؽڣ�����Ϊ������
//	    2�Ŷ��Ϊ�����ؽڣ�����Ϊ���°�
//		3�Ŷ��Ϊ��צ������Ϊ�ſ�
//		4�Ŷ��Ϊ�����壬����Ϊ��λ
uint16_t claw_pos = 470;
uint16_t claw_catch_pos = 410;
uint16_t claw_loose_pos = 510;
uint16_t claw_middle_pos = 470;
uint16_t paidanban_pos = 500;
uint16_t bogan_zhunbei_pos = 850;
uint16_t bogan_jiqiu_pos = 650;
uint16_t bogan_shouqi_pos = 250;


//--------------------------------������͹�ֶ������----------------------------
uint16_t huadao_vertical_pwm = 800;//900��ֱ600����(��Χ250-1250)
uint16_t huadao_slope_out_pwm = 770;
uint16_t huadao_slope_in_pwm = 1100;
uint16_t tulun_up_pwm = 1250;//1250����250����(��Χ250-1250)
uint16_t tulun_down_pwm = 250;

//--------------------------------��е����������----------------------------
// ���˶�ѧ����ṹ��
struct arm_solver solver;
uint8_t error;

//--------------------------------�����������--------------------------------
// ��װ���ؿ��Ƶĺ���
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque)
{
	modfiy_torque_cmd(&ctrl->unitree_send, 0, torque);
	UnitreeSend(&ctrl->unitree_send);
	ExtractData(&ctrl->unitree_recv, &motor_rx_temp);
}

// ��װ�ٶȿ��Ƶĺ���
void unitree_speed_ctrl(unitree_ctrl_t *ctrl, float speed, float kw)
{
	modfiy_speed_cmd(&ctrl->unitree_send, 0, speed, kw);
	UnitreeSend(&ctrl->unitree_send);
	ExtractData(&ctrl->unitree_recv, &motor_rx_temp);
}

//�������а�ȫ��ʿ������������޹���
void unitree_save_check(void)
{
	if(unitree_Data.unitree_recv.T >= UNITREE_MAX_TORQUE || unitree_Data.unitree_recv.T <= -UNITREE_MAX_TORQUE)
	{
		//��ǰ���ع�����������Ϊ��
		unitree_torque_ctrl(&unitree_Data, 0);
		dangerous_count ++;
	}
}


// �ٶ�pid����
void unitree_w_pid_ctrl(float w)
{
	float real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;

	// ��������ת
	if (unitree_pos_pid.error[0] > 0.01f)	//��0.01�����
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

// λ��pid����
void unitree_pos_pid_ctrl(float pos)
{
	if(pos > UNITREE_MAX_POS || pos < -UNITREE_MAX_POS)
		return;
	
	set_w = PID_calc(&unitree_pos_pid, unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose, pos);
	unitree_w_pid_ctrl(set_w);
}

//�ػ�������A1λ�ÿ���
void unitree_move(uint8_t flag, float pos, float w)
{
	if (flag == 1) // ����
	{
		// �ų����µ����
		if ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) > pos)
			return;
		// ��λ�����ٶ�ģʽ
		while ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) < (pos - 0.08f))
		{
			// ��������ǰ�����أ�wΪ1.2��kwΪ3
			modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, 0, up_w, 0, 3);
			UnitreeSend(&unitree_Data.unitree_send);
			ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
			osDelay(1);
		}
		// Сλ����λ��ģʽ��kp = 0.02 , kw = 3
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, (unitree_Data.zero_pose + pos), 0, 0.006, 3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
		osDelay(1);
	}
	if (flag == 2) // ����
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


//----------------------------------�������--------------------------------
//�ƶ��ؽ��ϵ����߶��
void servo_arm_move(float angle1, float angle2)
{
	float input1;
	float input2;
	
	input1 = 500.0f - angle1 / 0.24f;
	input2 = 500.0f + angle2 / 0.24f;
	
	moveServos(4, servo_Data.serial_servo_Time, 1, (uint16_t)input1, 2, (uint16_t)input2, 3, claw_pos, 4, paidanban_pos);
}

//��еצ��ȡ��0��ȡ��1�ɿ���2�м�λ��
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


//����б�ʵ�����2Ϊ���ڿۣ�1Ϊ����0Ϊ��ֱ
void huadao_control(bool_t is_put_ball)
{
	if(is_put_ball == 1)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_slope_out_pwm);//700����(��Χ250-1250)
	}
	else if(is_put_ball == 0)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_vertical_pwm);//800��ֱ
	}
	else
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, huadao_slope_in_pwm);//1200
	}
}	
//����͹�ֵ���
void tulun_control(bool_t is_up)
{
	if(is_up)
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, tulun_up_pwm);//1250����250����
	}
	else
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, tulun_down_pwm);
	}
	
}


//Բ�̻��������
void bogan_control(uint8_t mode)
{
	if(mode == 0)	//����
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_shouqi_pos);//1250
	}
	else if(mode == 1)	//����
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_jiqiu_pos);//800
	}
	else if(mode == 2)	//׼������
	{
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, bogan_zhunbei_pos);//250
	}
	
}


//���޸�
//--------------------------------�ܻ�е�ۺ���--------------------------------
void arm_solve(float end_angle, float x, float y)
{
	error = arm_solver_analyze(&solver, end_angle, x, y);
	if(!error)
	{
		// ����a0Ϊ������		
//		servo_arm_move(solver.a1, solver.a2);
	}
	else
	{
		
	}
}

// ��е�۳�ʼ������
void Arm_Init(void)
{
	// ���˶�ѧ�����ʼ��
	arm_solver_init(&solver, 140.0f, 24.38f, 358.13f, 150.0f, 188.17f);
	
	// �ö������ʼ��
	serial_servo_UART_Init();
	servo_Data.serial_servo_Time = 400;
	
	// �����ؽڵ����ʼ��
	unitree_Uart_Init(unitree_rx_buf[0], unitree_rx_buf[1], Unitree_RX_BUF_NUM);
	PID_init(&unitree_w_pid,   PID_POSITION, unitree_w_pid_K,   UNITREE_W_PID_MAX_OUT,   UNITREE_W_PID_MAX_IOUT);
	PID_init(&unitree_pos_pid, PID_POSITION, unitree_pos_pid_K, UNITREE_POS_PID_MAX_OUT, UNITREE_POS_PID_MAX_IOUT);
}
