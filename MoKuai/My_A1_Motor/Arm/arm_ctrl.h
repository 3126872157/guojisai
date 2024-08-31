#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"

#define UNITREE_W_PID_KP 0.02
#define UNITREE_W_PID_KI 0.003
#define UNITREE_W_PID_KD 0.003

#define UNITREE_POS_PID_KP 1.5
#define UNITREE_POS_PID_KI 0.05
#define UNITREE_POS_PID_KD 75.0

// �������
typedef struct
{
	float zero_pose; // ��ʼ�Ƕ�

	motor_send_t unitree_send;
	motor_recv_t unitree_recv;
} unitree_ctrl_t;



//// �ö��������
//typedef struct
//{
//	uint8_t servo_uart_Time;
//	uint16_t servo_uart_W1;
//	uint16_t servo_uart_W2;

//	uint8_t LobotRxData[16];
//} uart_servo_Data_1;

//// �������
//typedef struct
//{
//	uint16_t pos1;
//	uint16_t pos2;
//} servo_data_1;


// ��е�۳�ʼ��
void Arm_Init(void);
//���������װ���ؿ��Ƶĺ���
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque);
//����������ٶ�PID����
void unitree_w_pid_ctrl(float w);
//�������λ��PID����
void unitree_pos_pid_ctrl(float pos);
#endif
