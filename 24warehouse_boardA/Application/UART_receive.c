#include "UART_receive.h"


/*------------------------串口接收变量-----------------------------*/

//IC卡
uint8_t RX_IC_buff[IC_BUFF_SIZE];//IC卡数据包接收缓冲区
uint8_t IC_data;//当前识别到的IC卡内的数据(几行几列)


//C板陀螺仪
uint8_t RX_INS_buff[INS_BUFF_SIZE];//C板INS接收缓冲区
uint8_t RX_INS_data[4];//C板INS接收数据(uint8_t类型)
float rx_gyro;//C板INS接收数据(float类型)


//视觉
uint8_t RX_shijue_buff[SHIJUE_BUFF_SIZE];//视觉数据包接收缓冲区
shijue_Data shijue_data;//视觉数据帧：球x,y,distance, 二维码x,y,

union shijue_msg_t//联合体(用于数据类型转换)
{
	float after;
	uint8_t before[4];
} shijue_msg;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
	if(huart == &huart8)//视觉数据DMA空闲中断接收
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, RX_shijue_buff, SHIJUE_BUFF_SIZE);
		if(RX_shijue_buff[0] == 0xFF && RX_shijue_buff[25] == 0xFE)
		{
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+1];
			}
			shijue_data.ball_x=shijue_msg.after;
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+5];
			}
			shijue_data.ball_y=shijue_msg.after;
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+9];
			}
			shijue_data.ball_distance=shijue_msg.after;
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+13];
			}
			shijue_data.QR_x=shijue_msg.after;
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+17];
			}
			shijue_data.QR_y=shijue_msg.after;
			for(int i = 0; i < 4; i++)
			{
				shijue_msg.before[i] = RX_shijue_buff[i+21];
			}
			shijue_data.QR_code=shijue_msg.after;
		}
	}
	
	if(huart == &huart3)//IC卡数据接收中断
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart3, RX_IC_buff, IC_BUFF_SIZE);
		if(RX_IC_buff[0] == 0x04)
		{
			uint8_t X = 0;
			for(uint8_t i = 0;i < 21 ; i++) X^=RX_IC_buff[i];//数据校验
			X=~X;//数据校验
			if(X == RX_IC_buff[21]) IC_data = RX_IC_buff[15];//校验成功
		}
	}
	
	if(huart == &huart2)//C板INS数据空闲中断接收
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart2, RX_INS_buff, INS_BUFF_SIZE);
		if(RX_INS_buff[0] == 0xFE)
		{
			RX_INS_data[0] = RX_INS_buff[4];
			RX_INS_data[1] = RX_INS_buff[3];
			RX_INS_data[2] = RX_INS_buff[2];
			RX_INS_data[3] = RX_INS_buff[1];
			memcpy(&rx_gyro, RX_INS_data, 4);
		}
	}
	
	if(huart == &huart4)
	{
//		HAL_UARTEx_ReceiveToIdle_IT(&huart4, buff, SIZE);
	}
		
	
}






