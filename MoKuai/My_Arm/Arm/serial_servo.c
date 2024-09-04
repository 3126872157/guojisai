#include "stm32f4xx.h" // Device header
#include "serial_servo.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

// �꺯�� ���A�ĵͰ�λ
#define GET_LOW_BYTE(A) ((uint8_t)(A))
// �꺯�� ���A�ĸ߰�λ
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

uint8_t ServoTxBuf[128]; // ���߶�����ͻ���
uint8_t ServoRxBuf[20];	 // ���߶�����ջ���
uint16_t batteryVolt;

uint8_t Servo_Rx_Data[20];

// ��ʼ�����߶��
void serial_servo_UART_Init(void)
{
	__HAL_UART_ENABLE_IT(&SERIAL_SERVO_HUART, UART_IT_IDLE);	// ʹ�ܴ��ڿ����ж�
	HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, ServoRxBuf, 255); // ����DMA���䣬������1�����ݰ��˵�recvive_buff�У�ÿ��255���ֽ�
}

// ���ƶ������ת��
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0))
	{ // ���ID���ܴ���31,�ɸ��ݶ�Ӧ���ư��޸�
		return;
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // ���֡ͷ
	ServoTxBuf[2] = 8;							  // ���ݳ��ȣ����ƶ���ĸ��� * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;				  // ���ݳ���=Ҫ���ƶ����*3+5���˴�=1*3+5//������ƶ�ָ��
	ServoTxBuf[4] = 1;							  // Ҫ���ƵĶ������
	ServoTxBuf[5] = GET_LOW_BYTE(Time);			  // ȡ��ʱ��ĵͰ�λ
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);		  // ȡ��ʱ��ĸ߰�λ
	ServoTxBuf[7] = servoID;					  // ���ID
	ServoTxBuf[8] = GET_LOW_BYTE(Position);		  // ȡ��Ŀ��λ�õĵͰ�λ
	ServoTxBuf[9] = GET_HIGH_BYTE(Position);	  // ȡ��Ŀ��λ�õĸ߰�λ

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, 10);
}

// ���ƶ�����ת����param��������ʱ�䣻id + �Ƕȣ�id + �Ƕ� ...
void moveServos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr; // �ɱ�����б�

	va_start(arg_ptr, Time); // ȡ�ÿɱ�����׵�ַ
	if (Num < 1 || Num > 32)
	{
		return; // ���������Ϊ��ʹ���32��ʱ�䲻��С��0
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // ���֡ͷ
	ServoTxBuf[2] = Num * 3 + 5;				  // ���ݳ��� = Ҫ���ƶ���� * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;				  // ����ƶ�ָ��
	ServoTxBuf[4] = Num;						  // Ҫ���ƶ����
	ServoTxBuf[5] = GET_LOW_BYTE(Time);			  // ȡ��ʱ��ĵͰ�λ
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);		  // ȡ��ʱ��ĸ߰�λ

	for (i = 0; i < Num; i++)
	{								 // �ӿɱ������ȡ�ò�ѭ�������ID�Ͷ�ӦĿ��λ��
		temp = va_arg(arg_ptr, int); // �ɲ�����ȡ�ö��ID
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);						  // �ɱ������ȡ�ö�ӦĿ��λ��
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); // ���Ŀ��λ�õͰ�λ
		ServoTxBuf[index++] = GET_HIGH_BYTE(temp);			  // ���Ŀ��λ�ø߰�λ
	}

	va_end(arg_ptr); // �ÿ�arg_ptr

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, ServoTxBuf[2] + 2);
}

// ������������ȡ�������ĽǶȣ�param��������id��id ...
void getServosAngle(uint8_t Num, ...)
{
	uint8_t index = 5;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr; // �ɱ�����б�

	va_start(arg_ptr, Num); // ȡ�ÿɱ�����׵�ַ
	if (Num < 1 || Num > 32)
	{
		return; // ���������Ϊ��ʹ���32��ʱ�䲻��С��0
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // ���֡ͷ
	ServoTxBuf[2] = Num + 3;					  // ���ݳ��� = Ҫ���ƶ���� + 3
	ServoTxBuf[3] = CMD_MULT_SERVO_POS_READ;	  // �����ȡ�Ƕ�ָ��
	ServoTxBuf[4] = Num;						  // Ҫ���ƶ����

	for (i = 0; i < Num; i++)
	{								 // �ӿɱ������ȡ�ò�ѭ�������ID�Ͷ�ӦĿ��λ��
		temp = va_arg(arg_ptr, int); // �ɲ�����ȡ�ö��ID
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
	}

	va_end(arg_ptr); // �ÿ�arg_ptr

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, ServoTxBuf[2] + 2);
}

// �ö����߶��it.c�ļ�
void USER_SERIAL_SERVO_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart == &SERIAL_SERVO_HUART)
	{
		// ֹͣ����DMA����
		HAL_UART_DMAStop(&SERIAL_SERVO_HUART);

		// ������յ������ݳ���
		uint8_t data_length = SERIAL_SERVO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&SERIAL_SERVO_HDMA_RX);

		if (ServoRxBuf[0] != 0x55 || ServoRxBuf[1] != 0x55)
		{
			// ������ջ�����
			memset(ServoRxBuf, 0, data_length);

			// ������ʼDMA���� ÿ��255�ֽ�����
			HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
			return;
		}
		// ת������
		memcpy(Servo_Rx_Data, ServoRxBuf, data_length);

		// ������ջ�����
		memset(ServoRxBuf, 0, data_length);

		// ������ʼDMA���� ÿ��255�ֽ�����
		HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart == &SERIAL_SERVO_HUART)
//	{
//		// ֹͣ����DMA����
//		HAL_UART_DMAStop(&SERIAL_SERVO_HUART);

//		// ������յ������ݳ���
//		uint8_t data_length = SERIAL_SERVO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&SERIAL_SERVO_HDMA_RX);

//		if (ServoRxBuf[0] != 0x55 || ServoRxBuf[1] != 0x55)
//		{
//			// ������ջ�����
//			memset(ServoRxBuf, 0, data_length);

//			// ������ʼDMA���� ÿ��255�ֽ�����
//			HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
//			return;
//		}
//		// ת������
//		//memcpy(uart_servo_Data.LobotRxData,ServoRxBuf,data_length);

//		// ������ջ�����
//		memset(ServoRxBuf, 0, data_length);

//		// ������ʼDMA���� ÿ��255�ֽ�����
//		HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
//	}
//}

//void UART5_IRQHandler(void)
//{
//	/* USER CODE BEGIN UART5_IRQn 0 */

//	/* USER CODE END UART5_IRQn 0 */
//	HAL_UART_IRQHandler(&huart5);
//	/* USER CODE BEGIN UART5_IRQn 1 */
//	USER_SERIAL_SERVO_UART_IRQHandler();
//	/* USER CODE END UART5_IRQn 1 */
//}

//__weak void USER_SERIAL_SERVO_UART_IDLECallback(UART_HandleTypeDef *huart)
//{
//	return;
//}

//void USER_SERIAL_SERVO_UART_IRQHandler(void)
//{
//	if (RESET != __HAL_UART_GET_FLAG(&SERIAL_SERVO_HUART, UART_FLAG_IDLE))
//	{
//		// ��������жϱ�־�������һֱ���Ͻ����жϣ�
//		__HAL_UART_CLEAR_IDLEFLAG(&SERIAL_SERVO_HUART);
//		// �����жϴ�����
//		USER_SERIAL_SERVO_UART_IDLECallback(&SERIAL_SERVO_HUART);
//	}
//}
