#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"

// 宇树电机数据
typedef struct
{
	float zero_pose; // 初始角度

	motor_send_t unitree_send;
	motor_recv_t unitree_data_rx;
} motr_ctr_t;

//// 幻尔电机数据
//typedef struct
//{
//	uint8_t servo_uart_Time;
//	uint16_t servo_uart_W1;
//	uint16_t servo_uart_W2;

//	uint8_t LobotRxData[16];
//} uart_servo_Data_1;

//// 开环舵机
//typedef struct
//{
//	uint16_t pos1;
//	uint16_t pos2;
//} servo_data_1;

/***********************************************************************/
// 机械臂初始化
void Arm_Init(void);
#endif
