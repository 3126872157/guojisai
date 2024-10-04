#ifndef __UART_RECEIVE_H__
#define __UART_RECEIVE_H__

#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "struct_typedef.h"

#define SHIJUE_BUFF_SIZE 26//�Ӿ����ݰ���С
#define IC_BUFF_SIZE 22//IC�����ݰ���С
#define INS_BUFF_SIZE 5//C�����������ݰ���С


typedef struct
{
	float ball_x;
	float ball_y;
	float ball_distance;
	float QR_x;
	float QR_y;
	float QR_code;
}shijue_Data;





#endif


