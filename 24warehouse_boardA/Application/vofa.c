#include <string.h>
#include <stdio.h>
#include "stdarg.h"
#include "main.h"
#include "usart.h"

volatile uint8_t  usart_dma_tx_over = 1;

int myprintf(const char *format,...)
{
  va_list arg;
  static char SendBuff[200] = {0};
  int rv;
  while(!usart_dma_tx_over);//�ȴ�ǰһ��DMA�������
 
  va_start(arg,format);
  rv = vsnprintf((char*)SendBuff,sizeof(SendBuff)+1,(char*)format,arg);
  va_end(arg);
  
//  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)SendBuff,rv);
  usart_dma_tx_over = 0;//��0ȫ�ֱ�־��������ɺ�������1
 
  return rv;
}

//�ڴ��ڷ����жϻص���д
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART1)
//	{
//  		 usart_dma_tx_over = 1;
//	}
// 
//}
