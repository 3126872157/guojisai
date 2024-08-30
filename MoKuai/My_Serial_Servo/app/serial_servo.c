#include "stm32f4xx.h"                  // Device header
#include "serial_servo.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

//�꺯�� ���A�ĵͰ�λ
#define GET_LOW_BYTE(A) ((uint8_t)(A))
//�꺯�� ���A�ĸ߰�λ
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

extern UART_HandleTypeDef huart6;

uint8_t ServoTxBuf[128];  //���߶�����ͻ���
uint8_t ServoRxBuf[16];
uint16_t batteryVolt;

/*********************************************************************************
 * Function:  moveServo
 * Description�� ���Ƶ������ת��
 * Parameters:   sevoID:���ID��Position:Ŀ��λ��,Time:ת��ʱ��
                    ���IDȡֵ:0<=���ID<=31,Timeȡֵ: Time > 0
 * Return:       �޷���
 * Others:
 **********************************************************************************/
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0)) {  			//���ID���ܴ���31,�ɸ��ݶ�Ӧ���ư��޸�
		return;
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER;	//���֡ͷ
	ServoTxBuf[2] = 8;								//���ݳ��ȣ����ƶ���ĸ��� * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;           		//���ݳ���=Ҫ���ƶ����*3+5���˴�=1*3+5//������ƶ�ָ��
	ServoTxBuf[4] = 1;                        		//Ҫ���ƵĶ������
	ServoTxBuf[5] = GET_LOW_BYTE(Time);       		//ȡ��ʱ��ĵͰ�λ
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);      		//ȡ��ʱ��ĸ߰�λ
	ServoTxBuf[7] = servoID;                  		//���ID
	ServoTxBuf[8] = GET_LOW_BYTE(Position);   		//ȡ��Ŀ��λ�õĵͰ�λ
	ServoTxBuf[9] = GET_HIGH_BYTE(Position);  		//ȡ��Ŀ��λ�õĸ߰�λ

	HAL_UART_Transmit(&huart6, ServoTxBuf, 10, 100);
}
