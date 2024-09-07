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
  while(!usart_dma_tx_over);//等待前一次DMA发送完成
 
  va_start(arg,format);
  rv = vsnprintf((char*)SendBuff,sizeof(SendBuff)+1,(char*)format,arg);
  va_end(arg);
  
//  HAL_UART_Transmit_DMA(&huart1,(uint8_t *)SendBuff,rv);
  usart_dma_tx_over = 0;//清0全局标志，发送完成后重新置1
 
  return rv;
}

//在串口发送中断回调里写
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART1)
//	{
//  		 usart_dma_tx_over = 1;
//	}
// 
//}
