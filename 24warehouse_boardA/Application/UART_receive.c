#include "UART_receive.h"


/*------------------------���ڽ��ձ���-----------------------------*/

//IC��
uint8_t RX_IC_buff[IC_BUFF_SIZE];//IC�����ݰ����ջ�����
uint8_t IC_data;//��ǰʶ�𵽵�IC���ڵ�����(���м���)


//C��������
uint8_t RX_INS_buff[INS_BUFF_SIZE];//C��INS���ջ�����
uint8_t RX_INS_data[4];//C��INS��������(uint8_t����)
float rx_gyro;//C��INS��������(float����)


//�Ӿ�
uint8_t RX_shijue_buff[SHIJUE_BUFF_SIZE];//�Ӿ����ݰ����ջ�����
shijue_Data shijue_data;//�Ӿ�����֡����x,y,distance, ��ά��x,y,

union shijue_msg_t//������(������������ת��)
{
	float after;
	uint8_t before[4];
} shijue_msg;


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	
	if(huart == &huart8)//�Ӿ�����DMA�����жϽ���
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
	
	if(huart == &huart3)//IC�����ݽ����ж�
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart3, RX_IC_buff, IC_BUFF_SIZE);
		if(RX_IC_buff[0] == 0x04)
		{
			uint8_t X = 0;
			for(uint8_t i = 0;i < 21 ; i++) X^=RX_IC_buff[i];//����У��
			X=~X;//����У��
			if(X == RX_IC_buff[21]) IC_data = RX_IC_buff[15];//У��ɹ�
		}
	}
	
	if(huart == &huart2)//C��INS���ݿ����жϽ���
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






