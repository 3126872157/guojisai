#ifndef __ARM_STRUCT_H__
#define __ARM_STRUCT_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"
#include "servo.h"

//宇树电机数据
typedef struct{
float unitree_Torque;
float unitree_W;
float unitree_POS;
float unitree_KP;
float unitree_KW;
uint8_t unitree_MODE;
	
float pos0;//初始角度
	
MOTOR_recv unitree_data_rx;
MOTOR_send unitree_send;
	
uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];//双缓冲数组
ServoComdDataV3 motor_data_rx6;//接收缓存
}unitree_Data_1;

//幻尔电机数据
typedef struct{
uint8_t servo_uart_Time;
uint16_t servo_uart_W1;
uint16_t servo_uart_W2;
	
uint8_t LobotRxData[16];
uint8_t LobotRxBuf[16];   //接收缓存 
}uart_servo_Data_1;

//开环舵机
typedef struct{
	uint16_t pos1;
	uint16_t pos2;
}servo_data_1;



#endif