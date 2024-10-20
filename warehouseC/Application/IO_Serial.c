#include "IO_Serial.h"
#include "UART_receive.h"

//uint8_t Data;
//uint8_t recvData = 0;  //接收数据
//uint8_t recvStat = COM_STOP_BIT;  //接收状态

//IC卡，移到main.c
//uint8_t ic_read_start = 0;
//uint8_t ic_buf_size = 0;
//uint8_t ic_buf[22] = {0};
//uint8_t X = 0;
//uint8_t RX_IC_buff[IC_BUFF_SIZE];//IC卡数据包接收缓冲区
//uint8_t IC_data;//当前识别到的IC卡内的数据(几行几列)

void my_delay_us(volatile uint32_t nTime)
{ 
	uint16_t tmp;
	tmp = __HAL_TIM_GetCounter(&DELAY_TIM);//获得 TIM2 计数器的值
	if(tmp + nTime <= 65535)
	{		
		while( (__HAL_TIM_GetCounter(&DELAY_TIM) - tmp) < nTime ); 
	}
	else
	{
		__HAL_TIM_SET_COUNTER(&DELAY_TIM, 0);//设置 TIM3 计数器寄存器值为0
		while( __HAL_TIM_GetCounter(&DELAY_TIM) < nTime );
	}
}

//移到INS_TASK
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == Serial_RX_Pin)
//	{
//		if(IO_SERIAL_RX == 0) 
//		{
//			if(recvStat == COM_STOP_BIT)
//			{	
//				recvStat = COM_START_BIT;
//				//这要延时下 跳过起始位等待
//				my_delay_us(4);
//				//开启Time1中断计数
//				HAL_TIM_Base_Start_IT(&Serial_TIM);
//			}
//		}
//	}
//}

//移到main.c
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM6)
//	{
//		HAL_IncTick();
//	}
//	
//	if(htim == &Serial_TIM)    //如果是TIM1 触发中断
//	{
//		recvStat++;
//		if(recvStat == COM_STOP_BIT)
//		{
//			HAL_TIM_Base_Stop(&Serial_TIM);
//			__HAL_TIM_SetCounter(&Serial_TIM, 0);
//			
//			//到这就接收到完整的1个字节数据
//			if(ic_read_start == 1)
//			{
//				if(ic_buf_size == 1 && recvData != 0x16)	//第二帧对不上
//				{
//					ic_read_start = 0;
//					ic_buf_size = -1;
//				}
//				ic_buf[ic_buf_size++] = recvData;
//				
//				if(ic_buf_size == 22)						//接收完毕
//				{
//					ic_read_start = 0;
//					ic_buf_size = 0;						//计数归零
//					for(uint8_t i = 0;i < 21 ; i++)
//					{
//						X^=ic_buf[i];
//					}
//					X=~X;
//					if(X == ic_buf[21])						//校验成功
//					{
//						IC_data = ic_buf[15];				//将球的数据存入IC_Data
//					}
//					X = 0;
//				}
//			}
//		
//			if(ic_read_start == 0 && recvData == 0x04)		//检测到帧头
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
