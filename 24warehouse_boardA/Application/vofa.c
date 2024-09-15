#include "VOFA.h"


volatile uint8_t  usart_dma_tx_over = 1;


//���������͵�����ָ�룬������int my_vofa_printf(const char *format,...)
int my_vofa_printf(const char *format,...)
{
  va_list arg;
  static char send_buff[200] = {0};
  int rx_len;//���������ֽڳ���
  
  while(!usart_dma_tx_over);//�ȴ�ǰһ��DMA�������
 
  va_start(arg, format);
  rx_len = vsnprintf((char*)send_buff,sizeof(send_buff)+1,(char*)format,arg);
  va_end(arg);
  
  send_buff[rx_len++] = 0x00;send_buff[rx_len++] = 0x00;send_buff[rx_len++] = 0x80;send_buff[rx_len++] = 0x7F; //VOFA��JustFloatģʽ��֡β
  
  HAL_UART_Transmit_DMA(&huart8,(uint8_t *)send_buff, rx_len);
  usart_dma_tx_over = 0;//��0ȫ�ֱ�־��������ɺ�������1
 
  return rx_len;
}


//�ڴ��ڷ����жϻص���д
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==UART8)
//	{
//  		 usart_dma_tx_over = 1;
//	}
// 
//}

