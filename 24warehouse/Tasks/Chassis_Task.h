/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "main.h"
#include "pid.h"
#include "user_lib.h"

//���̵���ٶȻ�PID
#define M2006_MOTOR_SPEED_PID_KP 330.0f
#define M2006_MOTOR_SPEED_PID_KI 0.4f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
#define MAX_MOTOR_CAN_CURRENT 8000.0f

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//�����������Ƶ�ʣ�(��δʹ�������),�ڼ�����̼��ٶ�ʱʹ����
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//�����˶��������ǰ����ƽ�ƣ���ת�ٶȣ�ԭ��Ϊ���ڵ�������
#define NORMAL_MAX_CHASSIS_SPEED_X 50.0f	//2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y 50.0f    //1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Z 100.0f   //????
//�������̵������ٶ�
#define MAX_WHEEL_SPEED 70.0f

//m2006ת��ת���ɵ����ٶ�(cm/s)�ı�����
#define M2006_MOTOR_RPM_TO_VECTOR 0.01477712074814815f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_VECTOR

//�����ٶ����ĸ����Ӻϳɣ�������1/4
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//����������ĵĳߴ�,�Գ����Ӿ���380mm
#define MOTOR_DISTANCE_TO_CENTER 0.19f
//�ĸ�ȫ�����ٶȺϳ�ϵ��������֮���Ŷ�
#define OMNI_WHEEL_SPEED_COMPOSITION 0.7071067812f
//��ת��������,ȫ���ֳ����ķ������Բ��õ�
#define CHASSIS_WZ_SET_SCALE 0.5f

//�����˶�ģʽ
typedef enum
{
  CHASSIS_STOP,
  CHASSIS_INFRARED,
  CHASSIS_ULTRASONIC,
  CHASSIS_MOVE_AND_ROTATE,
  CHASSIS_V,
  CHASSIS_VX_MOVEY,
  CHASSIS_VY_MOVEX,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
	
//  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
	
  const fp32 *chassis_INS_angle;            	//��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;              	//���̿���״̬��
  chassis_mode_e last_chassis_mode;         	//�����ϴο���״̬��
	
  chassis_motor_t motor_chassis[4];          	//���̵������
	
  pid_type_def motor_speed_pid[4];             	//���̵���ٶ�pid
  pid_type_def motor_distance_pid;				//���̵��ƽ��pid
  
  pid_type_def chassis_angle_pid;              	//���̸���Ƕ�pid
  pid_type_def motor_gyro_pid;
  pid_type_def motor_move_gyro_pid;
  
  fp32 x;
  fp32 y;
  fp32 vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 x_set;
  fp32 y_set;
  fp32 vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;      //��������̨����ԽǶȣ���λ rad
  fp32 chassis_relative_angle_set;  //���������̨���ƽǶ�
  fp32 chassis_yaw_set;             

  fp32 vx_max_speed;  				//ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  				//���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  				//��������ٶ� ��λm/s
  fp32 vy_min_speed;  				//�ҷ�������ٶ� ��λm/s
  fp32 wz_max_speed;  				//��ת����ٶ�
  fp32 wz_min_speed;  				//��ת��С�ٶ�
  
} chassis_move_t;

//����������
void chassis_task(void const * argument);


#endif
