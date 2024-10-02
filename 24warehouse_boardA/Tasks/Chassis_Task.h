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
#include "Bodanpan_Task.h"
#include "bodanpan.h"
#include "VOFA.h"

//�ٶȻ�PID��4������ֿ�����
#define M2006_MOTOR1_SPEED_PID_KP 610.0f
#define M2006_MOTOR1_SPEED_PID_KI 0.01f
#define M2006_MOTOR1_SPEED_PID_KD 0.0f
#define M2006_MOTOR1_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR1_SPEED_PID_MAX_IOUT 500.0f

#define M2006_MOTOR2_SPEED_PID_KP 600.0f
#define M2006_MOTOR2_SPEED_PID_KI 0.01f
#define M2006_MOTOR2_SPEED_PID_KD 0.0f
#define M2006_MOTOR2_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR2_SPEED_PID_MAX_IOUT 500.0f

#define M2006_MOTOR3_SPEED_PID_KP 650.0f
#define M2006_MOTOR3_SPEED_PID_KI 0.01f
#define M2006_MOTOR3_SPEED_PID_KD 0.0f
#define M2006_MOTOR3_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR3_SPEED_PID_MAX_IOUT 500.0f

#define M2006_MOTOR4_SPEED_PID_KP 650.0f
#define M2006_MOTOR4_SPEED_PID_KI 0.01f
#define M2006_MOTOR4_SPEED_PID_KD 0.0f
#define M2006_MOTOR4_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR4_SPEED_PID_MAX_IOUT 500.0f


//λ�û�PID��ƽ��/���룩
#define M2006_MOTOR_DISTANCE_PID_KP 0.82f
#define M2006_MOTOR_DISTANCE_PID_KI 0.0f
#define M2006_MOTOR_DISTANCE_PID_KD 0.0f
#define M2006_MOTOR_DISTANCE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_DISTANCE_PID_MAX_IOUT 4000.0f


//�����ǽǶȻ�PID
#define M2006_MOTOR_GYRO_PID_KP 0.6f
#define M2006_MOTOR_GYRO_PID_KI 0.0f
#define M2006_MOTOR_GYRO_PID_KD 50.0f
#define M2006_MOTOR_GYRO_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_GYRO_PID_MAX_IOUT 4000.0f
#define MAX_MOTOR_CAN_CURRENT 1800.0f


//����ת��PID
#define M2006_MOTOR_MOVE_GYRO_PID_KP 10.0f
#define M2006_MOTOR_MOVE_GYRO_PID_KI 0.0f
#define M2006_MOTOR_MOVE_GYRO_PID_KD 100.0f
#define M2006_MOTOR_MOVE_GYRO_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_MOVE_GYRO_PID_MAX_IOUT 2000.0f
#define MAX_MOTOR_CAN_CURRENT 1800.0f

#define MAX_MOTOR_CAN_CURRENT 1800.0f


//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 1
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME (CHASSIS_CONTROL_TIME_MS/1000f)
//�����������Ƶ�ʣ��ڼ�����̼��ٶ�ʱʹ����
#define CHASSIS_CONTROL_FREQUENCE (1000.0f/CHASSIS_CONTROL_TIME_MS)

//�����˶��������ǰ����ƽ�ƣ���ת�ٶȣ�ԭ��Ϊ���ڵ�������
#define NORMAL_MAX_CHASSIS_SPEED_X 50.0f	//2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y 50.0f    //1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Z 100.0f   //????
//�������̵������ٶ�
#define MAX_WHEEL_SPEED 70.0f

//m2006ת��ת���ɵ����ٶ�(cm/s)�ı�����
#define M2006_MOTOR_RPM_TO_VECTOR 0.0181805130416f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_VECTOR

//�����ٶ����ĸ����Ӻϳɣ�������1/4
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//����������ĵĳߴ�,�Գ����Ӿ���380mm
#define MOTOR_DISTANCE_TO_CENTER 0.19f
//�ĸ�ȫ�����ٶȺϳ�ϵ�������Ŷ�����������Сת�Ӹ�Ϊֱ�Ǳߣ��ϳ���·��
#define OMNI_WHEEL_SPEED_COMPOSITION 1.414213562373f
//��ת��������,ȫ���ֳ����ķ������Բ��õ�
#define CHASSIS_WZ_SET_SCALE 0.5f
//m2006ת�����ת����������·�̵ı�����cm)�������ܳ���8192�ͼ��ٱ�
#define M2006_MOTOR_ECD_TO_DISTANCE 0.0001331580545f 

//�������
#define SLOWSTART_V_K 0.0001f	//0.001f
#define SLOWSTART_WZ_K 0.002f
#define SLOWSTART_MINDIS_V 10	//10
#define SLOWSTART_MINDIS_W 8

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
  const motor_measure_t *chassis_motor_measure; //���ԭʼ����ָ��
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
  pid_type_def motor_gyro_pid;					//���̵���ǶȻ�������ƫ��
  pid_type_def motor_move_gyro_pid;				//���̵��ת��
  
  fp32 x;							//������ʻ���
  fp32 y;
  fp32 vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 x_set;
  fp32 y_set;
  fp32 vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 gyro;   						//�������0ʱ�̵ĽǶ�
  fp32 gyro_set;					//���õĿ��ƽǶ�
  fp32 last_gyro;            

  fp32 vx_max_speed;  				//ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  				//���˷�������ٶ� ��λm/s
  fp32 vy_max_speed;  				//��������ٶ� ��λm/s
  fp32 vy_min_speed;  				//�ҷ�������ٶ� ��λm/s
  fp32 wz_max_speed;  				//��ת����ٶ�
  fp32 wz_min_speed;  				//��ת��С�ٶ�
  
  //�Ժ����ƣ���
  int Dis_fm_set;
  
} chassis_move_t;

//����������
void chassis_task(void const * argument);
void ramp_function(float *data_in,float data_out,float k);//б�º���

#endif
