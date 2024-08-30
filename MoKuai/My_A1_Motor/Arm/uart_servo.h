#ifndef __UART_SERVO_H__
#define __UART_SERVO_H__

#include "main.h"

#define uart_servo_BUFFER_SIZE 255
#define CMD_MULT_SERVO_POS_READ 0x15  //获取舵机角度 

#define FRAME_HEADER 0x55             //帧头
#define CMD_SERVO_MOVE 0x03           //舵机移动指令
#define CMD_ACTION_GROUP_RUN 0x06     //运行动作组指令
#define CMD_ACTION_GROUP_STOP 0x07    //停止动作组指令
#define CMD_ACTION_GROUP_SPEED 0x0B   //设置动作组运行速度
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //获取电池电压指令

//extern bool isUartRxCompleted;
extern uint8_t LobotTxBuf[128];  //发送缓存
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);

typedef struct _lobot_servo_ {  //舵机ID,舵机目标位置
	uint8_t ID;
	uint16_t Position;
} LobotServo;

//初始换串口（空闲中断）
void uart_servo_UART5_Init(uint8_t* rxBuf,uint8_t len);
//得到多个舵机角度
void getServosGyro(void);
//计算舵机角度
void CaluServosGyro(void);
//控制单个舵机转动
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
////控制多个舵机转动（通过数组控制）
//void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
////控制多个舵机转动（通过可变参数）
//void moveServos(uint8_t Num, uint16_t Time, ...);
////运行指定动作组
//void runActionGroup(uint8_t numOfAction, uint16_t Times);
////停止动作组运行
//void stopActionGroup(void);
////设定指定动作组的运行速度
//void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
////设置所有动作组的运行速度
//void setAllActionGroupSpeed(uint16_t Speed);
////发送获取电池电压命令
//void getBatteryVoltage(void);

#endif
