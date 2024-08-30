#include "stm32f4xx.h"                  // Device header
#include "serial_servo.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

//宏函数 获得A的低八位
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//宏函数 获得A的高八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

extern UART_HandleTypeDef huart6;

uint8_t ServoTxBuf[128];  //总线舵机发送缓存
uint8_t ServoRxBuf[16];
uint16_t batteryVolt;

/*********************************************************************************
 * Function:  moveServo
 * Description： 控制单个舵机转动
 * Parameters:   sevoID:舵机ID，Position:目标位置,Time:转动时间
                    舵机ID取值:0<=舵机ID<=31,Time取值: Time > 0
 * Return:       无返回
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0)) {  			//舵机ID不能打于31,可根据对应控制板修改
		return;
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER;	//填充帧头
	ServoTxBuf[2] = 8;								//数据长度：控制舵机的个数 * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;           		//数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
	ServoTxBuf[4] = 1;                        		//要控制的舵机个数
	ServoTxBuf[5] = GET_LOW_BYTE(Time);       		//取得时间的低八位
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);      		//取得时间的高八位
	ServoTxBuf[7] = servoID;                  		//舵机ID
	ServoTxBuf[8] = GET_LOW_BYTE(Position);   		//取得目标位置的低八位
	ServoTxBuf[9] = GET_HIGH_BYTE(Position);  		//取得目标位置的高八位

	HAL_UART_Transmit(&huart6, ServoTxBuf, 10, 100);
}
