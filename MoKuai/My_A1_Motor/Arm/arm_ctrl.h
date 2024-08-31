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

// 宇树电机
typedef struct
{
	float zero_pose; // 初始角度

	motor_send_t unitree_send;
	motor_recv_t unitree_recv;
} unitree_ctrl_t;



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


// 机械臂初始化
void Arm_Init(void);
//宇树电机封装力矩控制的函数
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque);
//宇树电机角速度PID控制
void unitree_w_pid_ctrl(float w);
//宇树电机位置PID控制
void unitree_pos_pid_ctrl(float pos);
#endif
