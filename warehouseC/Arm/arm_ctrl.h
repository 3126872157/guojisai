#ifndef __ARM_CTRL_H__
#define __ARM_CTRL_H__

#include "main.h"
#include "unitree_a1.h"
#include "serial_servo.h"
#include "unitree_a1.h"
#include "pid.h"
#include "cmsis_os.h"
#include "tim.h"
//-------------------------------����-------------------------------

#define UNITREE_W_PID_KP 0.02f
#define UNITREE_W_PID_KI 0.003f
#define UNITREE_W_PID_KD 0.0f
#define UNITREE_W_PID_MAX_OUT 2.0f
#define UNITREE_W_PID_MAX_IOUT 0.2f

#define UNITREE_POS_PID_KP 30.0f
#define UNITREE_POS_PID_KI 0.0f
#define UNITREE_POS_PID_KD 20.0f
#define UNITREE_POS_PID_MAX_OUT 2.0f
#define UNITREE_POS_PID_MAX_IOUT 0.5f	//�޳�IOUT���������

//��ȫ
#define UNITREE_MAX_TORQUE 2.0f
#define UNITREE_MAX_POS	   1.20f

#define PWM_SERVO_TIM htim1
#define BOGAN_CHANNEL TIM_CHANNEL_1
#define TULUN_CHANNEL TIM_CHANNEL_2
#define HUADAO_CHANNEL TIM_CHANNEL_3

// ��������ṹ��
typedef struct
{
	float zero_pose;	// ��ʼ�Ƕ�
	float last_w;
	float filted_w;		//��ͨ�˲����

	motor_send_t unitree_send;
	motor_recv_t unitree_recv;
} unitree_ctrl_t;


//-------------------------------���߶��-------------------------------
//���߶���ṹ��
typedef struct
{
	uint16_t serial_servo_Time;
	uint16_t serial_servo_angle1;
	uint16_t serial_servo_angle2;

	uint8_t ServoRxData[20];
} serial_servo_t;


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
//�ƶ��ؽ��ϵ����߶��
void servo_arm_move(float angle1, float angle2);
// ��е�۽��㷽������
void arm_solve(float end_angle, float x, float y);
//��еצ����
void claw_control(uint16_t pos);
//����pwm����
void huadao_control(bool_t is_put_ball);
//͹��pwm����
void tulun_control(bool_t is_up);
//Բ�̻����˿���
void bogan_control(uint8_t mode);
#endif
