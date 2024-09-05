#include "main.h"
// #include "tim.h"
#include "arm_ctrl.h"
// #include "uart_servo.h"
#include "unitree_a1.h"
#include "pid.h"

//----------------------�����������----------------------
// ���ڻ���
extern uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];
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
float Tf = 0.06;
float set_w = 0;
float kw = 5.0;
float up_w = 1.2;
float down_w = -0.2;
uint8_t dangerous_count = 0;

// uart_servo_Data_1 uart_servo_Data;
// servo_data_1 servo_data;


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

// ���������ʼ����¼���
void unitree_check_zero_pose(unitree_ctrl_t *ctrl)
{
	unitree_torque_ctrl(ctrl, 0);
	HAL_Delay(10);
	unitree_torque_ctrl(ctrl, 0);
	ctrl->zero_pose = ctrl->unitree_recv.Pos;
}

// �ٶ�pid����
void unitree_w_pid_ctrl(float w)
{
	float real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
	float tff;

	// ��������ת
	if (w >= 0)
	{
		tff = Tf;
//		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.LW, w));
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, 0, w, 0, kw);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
	}
	if (w < 0)
	{
		tff = -Tf;
//		unitree_torque_ctrl(&unitree_Data, tff + PID_calc(&unitree_w_pid, unitree_Data.unitree_recv.LW, w));
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, -Tf, 0, w, 0, kw);
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
			HAL_Delay(1);
		}
		// Сλ����λ��ģʽ��kp = 0.02 , kw = 3
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, +Tf, (unitree_Data.zero_pose + pos), 0, 0.006, 3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
		HAL_Delay(1);
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
			HAL_Delay(1);
		}
		modfiy_mix_cmd(&unitree_Data.unitree_send, 0, -Tf, (unitree_Data.zero_pose + pos), 0, 0.006, 3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_recv, &motor_rx_temp);
		HAL_Delay(1);
	}
}

void Arm_Init(void)
{
	// �ö���ʼ��
	//	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);

	// ������ʼ��
	unitree_Usart6_Init(Unitree_rx6_buf[0], Unitree_rx6_buf[1], Unitree_RX_BUF_NUM);
	PID_init(&unitree_w_pid, PID_POSITION, unitree_w_pid_K, 2.0f, 0.2f); 	// 0.07, 0.0001, 0.1
	PID_init(&unitree_pos_pid, PID_POSITION, unitree_pos_pid_K, 2.0f, 0.5f);
	unitree_check_zero_pose(&unitree_Data);
}
