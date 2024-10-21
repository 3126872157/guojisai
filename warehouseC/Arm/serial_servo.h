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

#define FRAME_HEADER 0x55	// ֡ͷ����������0x55
#define CMD_SERVO_MOVE 0x03 // ����ƶ�ָ�� �������+ʱ��(����)x2+���ID+�Ƕ�x2 ������ƶ����������������ظ���
							// ������� 2 �� 9 �Ŷ���� 800ms �� 2 ��ת�� 800 ��λ�ã�9 ��ת�� 800 ��λ�ã�
							// ֡ͷ  ֡ͷ 	���ݳ��ȣ��������� + ָ�� + ����	ָ�� 	�������			ʱ��		���ID		�Ƕ�		���ID		�Ƕ�
							// 0x55  0x55 			     0x0B 					0x03 	  0x02 		  0x20 0x03 	 0x02 	  0x20 0x03 	 0x09 	  0x20 0x03
#define CMD_SERVO_UNLOAD 0x14		 // ���ƶ�����������ж��
#define CMD_MULT_SERVO_POS_READ 0x15 // ��ȡ�������ĽǶ�λ��ֵ
#define CMD_GET_BATTERY_VOLTAGE 0x0F // ��ȡ���ư��ص�ѹָ��

//���߶�����ڳ�ʼ��
void serial_servo_UART_Init(void);
// ���ƶ������ת��
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time);
// ���ƶ�����ת��
void moveServos(uint8_t Num, uint16_t Time, ...);
// ������������ȡ�������ĽǶ�
void getServosAngle(uint8_t Num, ...);

#endif
