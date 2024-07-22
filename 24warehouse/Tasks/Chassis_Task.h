/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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

//速度环PID
#define M2006_MOTOR_SPEED_PID_KP 330.0f
#define M2006_MOTOR_SPEED_PID_KI 0.4f
#define M2006_MOTOR_SPEED_PID_KD 0.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 4000.0f

//位置环PID（平动/距离）
#define M2006_MOTOR_DISTANCE_PID_KP 4.6f
#define M2006_MOTOR_DISTANCE_PID_KI 0.0000028f
#define M2006_MOTOR_DISTANCE_PID_KD 0.0000006f
#define M2006_MOTOR_DISTANCE_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_DISTANCE_PID_MAX_IOUT 4000.0f

//陀螺仪角度环PID
#define M2006_MOTOR_GYRO_PID_KP 1.2f
#define M2006_MOTOR_GYRO_PID_KI 0.0f
#define M2006_MOTOR_GYRO_PID_KD 50.0f
#define M2006_MOTOR_GYRO_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_GYRO_PID_MAX_IOUT 2000.0f
#define MAX_MOTOR_CAN_CURRENT 8000.0f

//底盘转向环PID
#define M2006_MOTOR_MOVE_GYRO_PID_KP 10.0f
#define M2006_MOTOR_MOVE_GYRO_PID_KI 0.0f
#define M2006_MOTOR_MOVE_GYRO_PID_KD 100.0f
#define M2006_MOTOR_MOVE_GYRO_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M2006_MOTOR_MOVE_GYRO_PID_MAX_IOUT 2000.0f
#define MAX_MOTOR_CAN_CURRENT 8000.0f

#define MAX_MOTOR_CAN_CURRENT 8000.0f

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357
//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f
//底盘任务控制频率，(尚未使用这个宏),在计算底盘加速度时使用了
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//底盘运动过程最大前进，平移，旋转速度（原本为现在的两倍）
#define NORMAL_MAX_CHASSIS_SPEED_X 50.0f	//2.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y 50.0f    //1.5f
#define NORMAL_MAX_CHASSIS_SPEED_Z 100.0f   //????
//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 70.0f

//m2006转速转化成底盘速度(cm/s)的比例，
#define M2006_MOTOR_RPM_TO_VECTOR 0.01477712074814815f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M2006_MOTOR_RPM_TO_VECTOR

//底盘速度由四个轮子合成，所以是1/4
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
//电机到车中心的尺寸,对称轮子距离380mm
#define MOTOR_DISTANCE_TO_CENTER 0.19f
//四个全向轮速度合成系数，二分之根号二
#define OMNI_WHEEL_SPEED_COMPOSITION 0.7071067812f
//旋转修正因子,全向轮车四四方方所以不用的
#define CHASSIS_WZ_SET_SCALE 0.5f

//底盘运动模式
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
	
//  const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
	
  const fp32 *chassis_INS_angle;            	//获取陀螺仪解算出的欧拉角指针
  chassis_mode_e chassis_mode;              	//底盘控制状态机
  chassis_mode_e last_chassis_mode;         	//底盘上次控制状态机
	
  chassis_motor_t motor_chassis[4];          	//底盘电机数据
	
  pid_type_def motor_speed_pid[4];             	//底盘电机速度pid
  pid_type_def motor_distance_pid;				//底盘电机平动pid
  pid_type_def motor_gyro_pid;					//底盘电机角度环，修正偏航
  pid_type_def motor_move_gyro_pid;				//底盘电机转向环
  
  fp32 x;							//底盘行驶里程
  fp32 y;
  fp32 vx;                          //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                          //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 x_set;
  fp32 y_set;
  fp32 vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 gyro;   						//底盘相对0时刻的角度
  fp32 gyro_set;					//设置的控制角度
  fp32 last_gyro;            

  fp32 vx_max_speed;  				//前进方向最大速度 单位m/s
  fp32 vx_min_speed;  				//后退方向最大速度 单位m/s
  fp32 vy_max_speed;  				//左方向最大速度 单位m/s
  fp32 vy_min_speed;  				//右方向最大速度 单位m/s
  fp32 wz_max_speed;  				//旋转最大速度
  fp32 wz_min_speed;  				//旋转最小速度
  
  //稍后完善！！
  int Dis_fm_set;
  
} chassis_move_t;

//函数声明区
void chassis_task(void const * argument);


#endif
