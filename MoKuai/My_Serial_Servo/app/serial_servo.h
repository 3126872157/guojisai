#ifndef _SERISL_SERVO_H_
#define _SERISL_SERVO_H_

#include "stm32f4xx.h"                  // Device header
#include "usart.h"

#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令 舵机个数+时间(共用)x2+舵机ID+角度x2 （如控制多个舵机：后俩参数重复）
									  //控制 2 和 9 号舵机在 800ms 内 2 号转到 800 的位置，9 号转到 800 的位置：
									  //0x55 0x55 0x0B 0x03 0x02 0x20 0x03 0x02 0x20 0x03 0x09 0x20 0x03
#define CMD_SERVO_UNLOAD 0x14		  //控制多个舵机马达掉电卸力
#define CMD_MULT_SERVO_POS_READ 0x15  //读取多个舵机的角度位置值
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取控制板电池电压指令

void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);

#endif
