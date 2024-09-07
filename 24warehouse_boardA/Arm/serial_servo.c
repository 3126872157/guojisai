#include "stm32f4xx.h" // Device header
#include "serial_servo.h"
#include "usart.h"
#include <stdarg.h>
#include <string.h>

// 宏函数 获得A的低八位
#define GET_LOW_BYTE(A) ((uint8_t)(A))
// 宏函数 获得A的高八位
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))

uint8_t ServoTxBuf[128]; // 总线舵机发送缓存
uint8_t ServoRxBuf[20];	 // 总线舵机接收缓存
uint16_t batteryVolt;

uint8_t Servo_Rx_Data[20];

// 初始化总线舵机
void serial_servo_UART_Init(void)
{
	__HAL_UART_ENABLE_IT(&SERIAL_SERVO_HUART, UART_IT_IDLE);	// 使能串口空闲中断
	HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, ServoRxBuf, 255); // 设置DMA传输，将串口1的数据搬运到recvive_buff中，每次255个字节
}

// 控制舵机单个转动
void moveServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
	if (servoID > 31 || !(Time > 0))
	{ // 舵机ID不能打于31,可根据对应控制板修改
		return;
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // 填充帧头
	ServoTxBuf[2] = 8;							  // 数据长度：控制舵机的个数 * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;				  // 数据长度=要控制舵机数*3+5，此处=1*3+5//填充舵机移动指令
	ServoTxBuf[4] = 1;							  // 要控制的舵机个数
	ServoTxBuf[5] = GET_LOW_BYTE(Time);			  // 取得时间的低八位
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);		  // 取得时间的高八位
	ServoTxBuf[7] = servoID;					  // 舵机ID
	ServoTxBuf[8] = GET_LOW_BYTE(Position);		  // 取得目标位置的低八位
	ServoTxBuf[9] = GET_HIGH_BYTE(Position);	  // 取得目标位置的高八位

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, 10);
}

// 控制多个电机转动，param：数量，时间；id + 角度；id + 角度 ...
void moveServos(uint8_t Num, uint16_t Time, ...)
{
	uint8_t index = 7;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr; // 可变参数列表

	va_start(arg_ptr, Time); // 取得可变参数首地址
	if (Num < 1 || Num > 32)
	{
		return; // 舵机数不能为零和大与32，时间不能小于0
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // 填充帧头
	ServoTxBuf[2] = Num * 3 + 5;				  // 数据长度 = 要控制舵机数 * 3 + 5
	ServoTxBuf[3] = CMD_SERVO_MOVE;				  // 舵机移动指令
	ServoTxBuf[4] = Num;						  // 要控制舵机数
	ServoTxBuf[5] = GET_LOW_BYTE(Time);			  // 取得时间的低八位
	ServoTxBuf[6] = GET_HIGH_BYTE(Time);		  // 取得时间的高八位

	for (i = 0; i < Num; i++)
	{								 // 从可变参数中取得并循环填充舵机ID和对应目标位置
		temp = va_arg(arg_ptr, int); // 可参数中取得舵机ID
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
		temp = va_arg(arg_ptr, int);						  // 可变参数中取得对应目标位置
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp)); // 填充目标位置低八位
		ServoTxBuf[index++] = GET_HIGH_BYTE(temp);			  // 填充目标位置高八位
	}

	va_end(arg_ptr); // 置空arg_ptr

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, ServoTxBuf[2] + 2);
}

// 发送命令来读取多个舵机的角度，param：数量；id；id ...
void getServosAngle(uint8_t Num, ...)
{
	uint8_t index = 5;
	uint8_t i = 0;
	uint16_t temp;
	va_list arg_ptr; // 可变参数列表

	va_start(arg_ptr, Num); // 取得可变参数首地址
	if (Num < 1 || Num > 32)
	{
		return; // 舵机数不能为零和大与32，时间不能小于0
	}
	ServoTxBuf[0] = ServoTxBuf[1] = FRAME_HEADER; // 填充帧头
	ServoTxBuf[2] = Num + 3;					  // 数据长度 = 要控制舵机数 + 3
	ServoTxBuf[3] = CMD_MULT_SERVO_POS_READ;	  // 舵机读取角度指令
	ServoTxBuf[4] = Num;						  // 要控制舵机数

	for (i = 0; i < Num; i++)
	{								 // 从可变参数中取得并循环填充舵机ID和对应目标位置
		temp = va_arg(arg_ptr, int); // 可参数中取得舵机ID
		ServoTxBuf[index++] = GET_LOW_BYTE(((uint16_t)temp));
	}

	va_end(arg_ptr); // 置空arg_ptr

	HAL_UART_Transmit_DMA(&SERIAL_SERVO_HUART, ServoTxBuf, ServoTxBuf[2] + 2);
}

// 幻尔总线舵机it.c文件
void USER_SERIAL_SERVO_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if (huart == &SERIAL_SERVO_HUART)
	{
		// 停止本次DMA传输
		HAL_UART_DMAStop(&SERIAL_SERVO_HUART);

		// 计算接收到的数据长度
		uint8_t data_length = SERIAL_SERVO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&SERIAL_SERVO_HDMA_RX);

		if (ServoRxBuf[0] != 0x55 || ServoRxBuf[1] != 0x55)
		{
			// 清零接收缓冲区
			memset(ServoRxBuf, 0, data_length);

			// 重启开始DMA传输 每次255字节数据
			HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
			return;
		}
		// 转移数据
		memcpy(Servo_Rx_Data, ServoRxBuf, data_length);

		// 清零接收缓冲区
		memset(ServoRxBuf, 0, data_length);

		// 重启开始DMA传输 每次255字节数据
		HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
	}
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart == &SERIAL_SERVO_HUART)
//	{
//		// 停止本次DMA传输
//		HAL_UART_DMAStop(&SERIAL_SERVO_HUART);

//		// 计算接收到的数据长度
//		uint8_t data_length = SERIAL_SERVO_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&SERIAL_SERVO_HDMA_RX);

//		if (ServoRxBuf[0] != 0x55 || ServoRxBuf[1] != 0x55)
//		{
//			// 清零接收缓冲区
//			memset(ServoRxBuf, 0, data_length);

//			// 重启开始DMA传输 每次255字节数据
//			HAL_UART_Receive_DMA(&SERIAL_SERVO_HUART, (uint8_t *)ServoRxBuf, 255);
//			return;
//		}
//		// 转移数据
//		//memcpy(uart_servo_Data.LobotRxData,ServoRxBuf,data_length);

//		// 清零接收缓冲区
//		memset(ServoRxBuf, 0, data_length);

//		// 重启开始DMA传输 每次255字节数据
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
//		// 清除空闲中断标志（否则会一直不断进入中断）
//		__HAL_UART_CLEAR_IDLEFLAG(&SERIAL_SERVO_HUART);
//		// 调用中断处理函数
//		USER_SERIAL_SERVO_UART_IDLECallback(&SERIAL_SERVO_HUART);
//	}
//}
