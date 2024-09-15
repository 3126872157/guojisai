#ifndef __BODANPAN_TASK_H
#define __BODANPAN_TASK_H

//#include "Servo_Control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "struct_typedef.h"
#include "main.h"
#include "math.h"
#include "pid.h"
//#include "Uart_Task.h"
#include "tim.h"
//#include "remote_control.h"
#include "bodanpan.h"
#include "Chassis_Task.h"


//拨蛋盘电机角度环pid
#define BODANPAN_ANGLE_KP 0.12f
#define BODANPAN_ANGLE_KI 0.0004f
#define BODANPAN_ANGLE_KD 0.0001f
#define M2006_MOTOR_ANGLE_PAN_PID_MAX_IOUT 4000.0f
#define M2006_MOTOR_ANGLE_PAN_PID_MAX_OUT 6000.0f

//拨蛋盘电机速度环pid
#define BODANPAN_SPEED_KP 10.0f
#define BODANPAN_SPEED_KI 1.0f
#define BODANPAN_SPEED_KD 0.0f
#define M2006_MOTOR_SPEED_PAN_PID_MAX_IOUT 3000.0f
#define M2006_MOTOR_SPEED_PAN_PID_MAX_OUT 4000.0f




void bodanpan_motor_init(void);
void bodanpan_motor_control(void);
void Bodanpan_Task(void const * argument);
	
#endif



