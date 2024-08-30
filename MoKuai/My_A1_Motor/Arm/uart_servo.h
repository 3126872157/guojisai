#ifndef __UART_SERVO_H__
#define __UART_SERVO_H__

#include "main.h"

#define uart_servo_BUFFER_SIZE 255
#define CMD_MULT_SERVO_POS_READ 0x15  //��ȡ����Ƕ� 

#define FRAME_HEADER 0x55             //֡ͷ
#define CMD_SERVO_MOVE 0x03           //����ƶ�ָ��
#define CMD_ACTION_GROUP_RUN 0x06     //���ж�����ָ��
#define CMD_ACTION_GROUP_STOP 0x07    //ֹͣ������ָ��
#define CMD_ACTION_GROUP_SPEED 0x0B   //���ö����������ٶ�
#define CMD_GET_BATTERY_VOLTAGE 0x0F  //��ȡ��ص�ѹָ��

//extern bool isUartRxCompleted;
extern uint8_t LobotTxBuf[128];  //���ͻ���
extern uint8_t LobotRxBuf[16];
extern uint16_t batteryVolt;
extern void receiveHandle(void);

typedef struct _lobot_servo_ {  //���ID,���Ŀ��λ��
	uint8_t ID;
	uint16_t Position;
} LobotServo;

//��ʼ�����ڣ������жϣ�
void uart_servo_UART5_Init(uint8_t* rxBuf,uint8_t len);
//�õ��������Ƕ�
void getServosGyro(void);
//�������Ƕ�
void CaluServosGyro(void);
//���Ƶ������ת��
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
////���ƶ�����ת����ͨ��������ƣ�
//void moveServosByArray(LobotServo servos[], uint8_t Num, uint16_t Time);
////���ƶ�����ת����ͨ���ɱ������
//void moveServos(uint8_t Num, uint16_t Time, ...);
////����ָ��������
//void runActionGroup(uint8_t numOfAction, uint16_t Times);
////ֹͣ����������
//void stopActionGroup(void);
////�趨ָ��������������ٶ�
//void setActionGroupSpeed(uint8_t numOfAction, uint16_t Speed);
////�������ж�����������ٶ�
//void setAllActionGroupSpeed(uint16_t Speed);
////���ͻ�ȡ��ص�ѹ����
//void getBatteryVoltage(void);

#endif
