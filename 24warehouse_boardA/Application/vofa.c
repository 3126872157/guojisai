#include "VOFA.h"


volatile uint8_t  usart_dma_tx_over = 1;


//参数：发送的数据指针，可填多个int my_vofa_printf(const char *format,...)
int my_vofa_printf(const char *format,...)
{
  va_list arg;
  static char send_buff[200] = {0};
  int rx_len;//发送数据字节长度
  
  while(!usart_dma_tx_over);//等待前一次DMA发送完成
 
  va_start(arg, format);
  rx_len = vsnprintf((char*)send_buff,sizeof(send_buff)+1,(char*)format,arg);
  va_end(arg);
  
  send_buff[rx_len++] = 0x00;send_buff[rx_len++] = 0x00;send_buff[rx_len++] = 0x80;send_buff[rx_len++] = 0x7F; //VOFA中JustFloat模式的帧尾
  
  HAL_UART_Transmit_DMA(&huart8,(uint8_t *)send_buff, rx_len);
  usart_dma_tx_over = 0;//清0全局标志，发送完成后重新置1
 
  return rx_len;
}


//在串口发送中断回调里写
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==UART8)
//	{
//  		 usart_dma_tx_over = 1;
//	}
// 
//}

