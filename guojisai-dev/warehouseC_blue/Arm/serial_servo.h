#ifndef _SERISL_SERVO_H_
#define _SERISL_SERVO_H_

#include "stm32f4xx.h" // Device header

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

#define SERIAL_SERVO_HUART huart6
#define SERIAL_SERVO_HDMA_RX hdma_usart6_rx
#define SERIAL_SERVO_HDMA_TX hdma_usart6_tx
#define SERIAL_SERVO_BUFFER_SIZE 255

#define FRAME_HEADER 0x55	// 帧头，两个都是0x55
#define CMD_SERVO_MOVE 0x03 // 舵机移动指令 舵机个数+时间(共用)x2+舵机ID+角度x2 （如控制多个舵机：后俩参数重复）
							// 例如控制 2 和 9 号舵机在 800ms 内 2 号转到 800 的位置，9 号转到 800 的位置：
							// 帧头  帧头 	数据长度（参数个数 + 指令 + 本身）	指令 	舵机个数			时间		舵机ID		角度		舵机ID		角度
							// 0x55  0x55 			     0x0B 					0x03 	  0x02 		  0x20 0x03 	 0x02 	  0x20 0x03 	 0x09 	  0x20 0x03
#define CMD_SERVO_UNLOAD 0x14		 // 控制多个舵机马达掉电卸力
#define CMD_MULT_SERVO_POS_READ 0x15 // 读取多个舵机的角度位置值
#define CMD_GET_BATTERY_VOLTAGE 0x0F // 获取控制板电池电压指令

//总线舵机串口初始化
void serial_servo_UART_Init(void);
// 控制舵机单个转动
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
// 控制多个电机转动
void moveServos(uint8_t Num, uint16_t Time, ...);
// 发送命令来读取多个舵机的角度
void getServosAngle(uint8_t Num, ...);

#endif
