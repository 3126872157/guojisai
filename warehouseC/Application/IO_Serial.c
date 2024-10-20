#include "IO_Serial.h"
#include "UART_receive.h"

//uint8_t Data;
//uint8_t recvData = 0;  //��������
//uint8_t recvStat = COM_STOP_BIT;  //����״̬

//IC�����Ƶ�main.c
//uint8_t ic_read_start = 0;
//uint8_t ic_buf_size = 0;
//uint8_t ic_buf[22] = {0};
//uint8_t X = 0;
//uint8_t RX_IC_buff[IC_BUFF_SIZE];//IC�����ݰ����ջ�����
//uint8_t IC_data;//��ǰʶ�𵽵�IC���ڵ�����(���м���)

void my_delay_us(volatile uint32_t nTime)
{ 
	uint16_t tmp;
	tmp = __HAL_TIM_GetCounter(&DELAY_TIM);//��� TIM2 ��������ֵ
	if(tmp + nTime <= 65535)
	{		
		while( (__HAL_TIM_GetCounter(&DELAY_TIM) - tmp) < nTime ); 
	}
	else
	{
		__HAL_TIM_SET_COUNTER(&DELAY_TIM, 0);//���� TIM3 �������Ĵ���ֵΪ0
		while( __HAL_TIM_GetCounter(&DELAY_TIM) < nTime );
	}
}

//�Ƶ�INS_TASK
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == Serial_RX_Pin)
//	{
//		if(IO_SERIAL_RX == 0) 
//		{
//			if(recvStat == COM_STOP_BIT)
//			{	
//				recvStat = COM_START_BIT;
//				//��Ҫ��ʱ�� ������ʼλ�ȴ�
//				my_delay_us(4);
//				//����Time1�жϼ���
//				HAL_TIM_Base_Start_IT(&Serial_TIM);
//			}
//		}
//	}
//}

//�Ƶ�main.c
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM6)
//	{
//		HAL_IncTick();
//	}
//	
//	if(htim == &Serial_TIM)    //�����TIM1 �����ж�
//	{
//		recvStat++;
//		if(recvStat == COM_STOP_BIT)
//		{
//			HAL_TIM_Base_Stop(&Serial_TIM);
//			__HAL_TIM_SetCounter(&Serial_TIM, 0);
//			
//			//����ͽ��յ�������1���ֽ�����
//			if(ic_read_start == 1)
//			{
//				if(ic_buf_size == 1 && recvData != 0x16)	//�ڶ�֡�Բ���
//				{
//					ic_read_start = 0;
//					ic_buf_size = -1;
//				}
//				ic_buf[ic_buf_size++] = recvData;
//				
//				if(ic_buf_size == 22)						//�������
//				{
//					ic_read_start = 0;
//					ic_buf_size = 0;						//��������
//					for(uint8_t i = 0;i < 21 ; i++)
//					{
//						X^=ic_buf[i];
//					}
//					X=~X;
//					if(X == ic_buf[21])						//У��ɹ�
//					{
//						IC_data = ic_buf[15];				//��������ݴ���IC_Data
//					}
//					X = 0;
//				}
//			}
//		
//			if(ic_read_start == 0 && recvData == 0x04)		//��⵽֡ͷ
//			{
//				ic_read_start = 1;
//				ic_buf[ic_buf_size++] = recvData;
//			}
//			return;
//		}
//		
//		if(IO_SERIAL_RX)
//		{
//			recvData |= (1 << (recvStat - 1));
//		}
//		else
//		{
//			recvData &= ~(1 << (recvStat - 1));
//		}
//	}
//}
