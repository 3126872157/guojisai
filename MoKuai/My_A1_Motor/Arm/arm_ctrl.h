#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"

// �����������
typedef struct
{
	float zero_pose; // ��ʼ�Ƕ�

	motor_send_t unitree_send;
	motor_recv_t unitree_data_rx;
} motr_ctr_t;

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

/***********************************************************************/
// ��е�۳�ʼ��
void Arm_Init(void);
#endif
