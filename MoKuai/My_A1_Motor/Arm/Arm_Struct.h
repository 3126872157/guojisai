#ifndef __ARM_STRUCT_H__
#define __ARM_STRUCT_H__

#include "main.h"
#include "uart_servo.h"
#include "unitree_a1.h"
#include "servo.h"

//???????????
typedef struct{
float unitree_Torque;
float unitree_W;
float unitree_POS;
float unitree_KP;
float unitree_KW;
uint8_t unitree_MODE;
	
float pos0;//??????
	
MOTOR_recv unitree_data_rx;
MOTOR_send unitree_send;
}unitree_Data_1;

//??????????
typedef struct{
uint8_t servo_uart_Time;
uint16_t servo_uart_W1;
uint16_t servo_uart_W2;
	
uint8_t LobotRxData[16];
}uart_servo_Data_1;

//???????
typedef struct{
	uint16_t pos1;
	uint16_t pos2;
}servo_data_1;

/***********************************************************************/
//??§Ö??????
void Arm_Init(void);
#endif

