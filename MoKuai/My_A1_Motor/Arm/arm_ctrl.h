#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"

//-------------------------------����-------------------------------
//PID����
#define UNITREE_W_PID_KP 0.02
#define UNITREE_W_PID_KI 0.003
#define UNITREE_W_PID_KD 0.003

#define UNITREE_POS_PID_KP 0.75	//1.5
#define UNITREE_POS_PID_KI 0.00 //0.05
#define UNITREE_POS_PID_KD 10.0 //75

//��ȫ
#define UNITREE_MAX_TORQUE 2.0f
#define UNITREE_MAX_POS	   2.0f

// �������
typedef struct
{
	float zero_pose;	// ��ʼ�Ƕ�
	float last_w;
	float filted_w;		//��ͨ�˲����

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
//�������а�ȫ��ʿ������������޹���
void unitree_save_check(void);
//����������ٶ�PID����
void unitree_w_pid_ctrl(float w);
//�������λ��PID����
void unitree_pos_pid_ctrl(float pos);
//�ػ�������A1λ�ÿ���
void unitree_move(uint8_t flag, float pos, float w);
#endif
