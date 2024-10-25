#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "unitree_a1.h"
#include "serial_servo.h"
#include "unitree_a1.h"
#include "pid.h"
#include "cmsis_os.h"
#include "tim.h"
//-------------------------------宇树-------------------------------

#define UNITREE_W_PID_KP 0.02f
#define UNITREE_W_PID_KI 0.003f
#define UNITREE_W_PID_KD 0.0f
#define UNITREE_W_PID_MAX_OUT 2.0f
#define UNITREE_W_PID_MAX_IOUT 0.2f

#define UNITREE_POS_PID_KP 30.0f
#define UNITREE_POS_PID_KI 0.0f
#define UNITREE_POS_PID_KD 20.0f
#define UNITREE_POS_PID_MAX_OUT 2.0f
#define UNITREE_POS_PID_MAX_IOUT 0.5f	//赞成IOUT来静差调整

//安全
#define UNITREE_MAX_TORQUE 2.0f
#define UNITREE_MAX_POS	   1.20f

#define PWM_SERVO_TIM htim1
#define BOGAN_CHANNEL TIM_CHANNEL_1
#define TULUN_CHANNEL TIM_CHANNEL_2
#define HUADAO_CHANNEL TIM_CHANNEL_3

// 宇树电机结构体
typedef struct
{
	float zero_pose;	// 初始角度
	float last_w;
	float filted_w;		//低通滤波后的

	motor_send_t unitree_send;
	motor_recv_t unitree_recv;
} unitree_ctrl_t;


//-------------------------------总线舵机-------------------------------
//总线舵机结构体
typedef struct
{
	uint16_t serial_servo_Time;
	uint16_t serial_servo_angle1;
	uint16_t serial_servo_angle2;

	uint8_t ServoRxData[20];
} serial_servo_t;


// 机械臂初始化
void Arm_Init(void);
//宇树电机封装力矩控制的函数
void unitree_torque_ctrl(unitree_ctrl_t *ctrl, float torque);
//宇树运行安全卫士：检测力矩有无过大
void unitree_save_check(void);
//宇树电机角速度PID控制
void unitree_w_pid_ctrl(float w);
//宇树电机位置PID控制
void unitree_pos_pid_ctrl(float pos);
//特化的宇树A1位置控制
void unitree_move(uint8_t flag, float pos, float w);
//移动关节上的总线舵机
void servo_arm_move(float angle1, float angle2);
// 机械臂解算方法控制
void arm_solve(float end_angle, float x, float y);
//机械爪控制
void claw_control(uint16_t pos);
//滑道pwm控制
void huadao_control(bool_t is_put_ball);
//凸轮pwm控制
void tulun_control(bool_t is_up);
//圆盘机拨杆控制
void bogan_control(uint8_t mode);
#endif
