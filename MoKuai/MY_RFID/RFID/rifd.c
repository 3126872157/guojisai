/*
M3650A-HAЭ��

���ݰ���ʽ:
  0			1		   2 			 3 		   4 				5-20 			  21
������	  ������		��������			��ַ 	  ״̬ 			   ������Ϣ 			У���
 0x04 	   0x16       0x03 			0x20 	0x00:�ɹ� 	      16�ֽ����� 		   X

���������
0x04 0x16 0x03 0x20 0x00 16�ֽ����� XУ���

У��ͣ�
X=ǰ21�ֽ����ΰ�λ��������ȡ��

���飺
���������ϴӿͷ����õ����ϣ��Ķ��ֲ�����ö�д���������
��������������ò����ʣ�д������ݵȵ�
*/


#include "stm32f4xx.h"                  // Device header
#include "usart.h"

#define RFID_RX_BUF_MAX_LEN 30

//����
uint8_t Cmd_Read_Id[8] = {0x01,0x08,0xa1,0x20,0x00,0x00,0x00,0x00};
uint8_t Cmd_Read_Block[8]	= {0x01,0x08,0xa3,0x20,0x00,0x00,0x00,0x00};
uint8_t Cmd_Write_Block[23] = {0x01,0x17,0xa4,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Cmd_Set_Work_Mode[8] = {0x03,0x08,0xC1,0x20,0x03,0x01,0x00,0x00};
uint8_t Cmd_Set_AutoRead[8] = {0x03,0x08,0xC8,0x20,0x01,0x00,0x00,0x00};

//���ڽ������
uint8_t buf[1] = {0};
uint8_t rfid_rx_buf[30] = {0};
uint8_t rfid_rx_flag = 0;		//ָʾ���������������֡
uint8_t rfid_error = 0;			//ָʾ���ڽ��պ��������ش�ʧ��
//ic�����ڽ������
uint8_t ic_read_start = 0;
uint8_t ic_buf[22] = {0};
uint8_t ic_buf_size = 0;
uint8_t X = 0;

//��ʼ��
void rfid_init(void)
{
	
	HAL_UART_Receive_IT(&huart1, buf, 1);
}

//���ڽ��պ���������ģʽ��³���Խϲ������
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t rfid_rx_count = 0;
	if(huart->Instance == USART1)
	{
		
		/********************����ģʽ*******************/
		rfid_rx_buf[rfid_rx_count] = buf[0];
		rfid_rx_count ++;
		if(rfid_rx_count >= RFID_RX_BUF_MAX_LEN)
		{
			rfid_rx_count = 0;
			rfid_error = 1;		//���²�����
		}
		if(rfid_rx_buf[1] == rfid_rx_count)	
		{
			rfid_rx_count = 0;	//���������ִ�����군��
			rfid_rx_flag = 1;
		}
		/********************����ģʽ*******************/
		
		
		/********************����ģʽ*******************/
		if(ic_read_start == 1)
		{
			if(ic_buf_size == 1 && buf[0] != 0x16)		//�ڶ�֡�Բ���
			{
				ic_read_start = 0;
				ic_buf_size = -1;
			}
			ic_buf[ic_buf_size++] = buf[0];
			
			if(ic_buf_size == 22)						//�������
			{
				ic_read_start = 0;
				ic_buf_size = 0;						//��������
				for(uint8_t i = 0;i < 21 ; i++)
				{
					X^=ic_buf[i];
				}
				X=~X;
				if(X == ic_buf[21])						//У��ɹ�
				{
					uint8_t flag = 1;
					for(uint8_t i=0;i<9;i++)			//�ж������Ƿ��Ѿ����
					{
						if(ic_buf[15]==bodanpan.date[i])
						{
							flag = 0;
						}
					}
					if(flag)							//��ic����
					{
						uint8_t which_box = 0;
						if(bodanpan.position == 0) which_box = 8;
						else which_box = bodanpan.position - 1;
						bodanpan.date[which_box] = ic_buf[15];
						bodanpan.box_state[which_box] = 1;
					}
				}
				X = 0;
			}
		}
		
		if(ic_read_start == 0 && buf[0] == 0x04)//��⵽֡ͷ
		{
			ic_read_start = 1;
			ic_buf[ic_buf_size++] = buf[0];
		}
		/********************����ģʽ*******************/
		HAL_UART_Receive_IT(&huart1, buf, 1);
	}
}

//RxģʽУ����
unsigned char RxCheckSum(unsigned char *ptr,unsigned char len)
{
	unsigned char i;
	unsigned char checksum;
	checksum = 0;
	for(i=0;i<(len-1);i++)
	{
		   checksum ^= ptr[i];
	}
	checksum = ~checksum;
	if(ptr[len-1] == checksum)
		return 	1;	//У��OK
	else 
		return 	0;	//У��ERROR
}

//TxģʽCRC��֤����
void TxCheckSum(unsigned char *ptr,unsigned char len)
{
	unsigned char i;
	unsigned char checksum;
	checksum = 0;
	for(i=0;i<(len-1);i++)
	{
		   checksum ^= ptr[i];
	}
	checksum = ~checksum;
	ptr[len-1] = checksum;
}

//��ȡID������2λ�����ͣ�4λ����
uint8_t rfid_ReadId(unsigned char *IDout)
{
	uint8_t status;
	uint8_t i;
	Cmd_Read_Id[5] = 0x01;											//������������ʾ
	TxCheckSum(Cmd_Read_Id, Cmd_Read_Id[1]);						//����У���
	HAL_UART_Transmit(&huart1, Cmd_Read_Id, Cmd_Read_Id[1], 100);	//���Ͷ�����ID����	 
	//�ȴ�ģ�鷵�����ݣ�����1.5MS
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf, rfid_rx_buf[1]);			//�Խ��յ�������У��
		if(status != 1)  											//�ж�У����Ƿ���ȷ
		{
			return 2;												//У��λʧ�ܷ���2
		}
		status = rfid_rx_buf[4];
		if(status != 1)												//�ж��Ƿ���ȷ�Ķ�����
		{
		 	return 3;												//��֪��ôʧ�ܵķ���3
		}
		if((rfid_rx_buf[0] == 0x01)&&(rfid_rx_buf[2] == 0xa1))		//�ж��Ƿ�Ϊ�����ŷ��ص����ݰ�
		{
			for(i=0;i<6;i++)										//��ȡ����ID��6�ֽ�		 
			{
				IDout[i] = rfid_rx_buf[i+5];						//������ĵ�5���ֽڿ�ʼΪ���ţ�����Ϊ6�ֽ�
			}
			return 0;		 										//�ɹ�����0
		}
 	} 
	return 1;														//���ڶ�ȡʧ�ܷ���1
}

//ָ���
uint8_t rfid_ReadFromBlock(uint8_t *dataout,uint8_t block)
{
	uint8_t status;
	uint8_t i;
	Cmd_Read_Block[4] = block;
	Cmd_Read_Block[5] = 0x01;										//������������ʾ
	TxCheckSum(Cmd_Read_Block, Cmd_Read_Block[1]);					//����У��
	HAL_UART_Transmit(&huart1, Cmd_Read_Block, Cmd_Read_Block[1], 100);	//���Ͷ�����ID����	
	//�ȴ�ģ�鷵�����ݣ�����1.5ms
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf,rfid_rx_buf[1]);			//�Խ��յ�������У��
		if(status != 1)		 										//�ж�У����Ƿ���ȷ
		{
			return 	2;												//У��λʧ�ܷ���2
		}
		status = rfid_rx_buf[4];									//��ȡ���ذ�״̬
		if(status != 1)												//�ж��Ƿ���ȷ�Ķ�����
		{
			return 3;
		}
		if((rfid_rx_buf[0] == 0x01)&&(rfid_rx_buf[2] == 0xa3))		//�ж��Ƿ�Ϊ�������ݷ��ص����ݰ�
		{
			for(i=0;i<16;i++)										//��ȡ�����ݣ�16�ֽ�	��һ�����ݿ�Ĵ�СΪ16�ֽ�	 
			{
				dataout[i] = rfid_rx_buf[i+5];						//������ĵ�5���ֽڿ�ʼΪ���ݣ�����Ϊ16�ֽ�
			}
			return 0;		 										//�ɹ�����0
		}
	}
	return 1;														//ʧ�ܷ���1
}

//�����Զ�������ʽ��1Ϊһֱ����0Ϊ��һ��
uint8_t rfid_Set_AutoRead(uint8_t autoread)						
{
	uint8_t status;
	Cmd_Set_AutoRead[4] = autoread;
	TxCheckSum(rfid_rx_buf, rfid_rx_buf[1]);						//����У��
	HAL_UART_Transmit(&huart1, Cmd_Set_AutoRead, Cmd_Set_AutoRead[1], 100);	//���Ͷ�����ID����	
	//�ȴ�ģ�鷵�����ݣ�����1.5ms
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf,rfid_rx_buf[1]);			//�Խ��յ�������У��
		if(status != 1)		 										//�ж�У����Ƿ���ȷ
		{
			return 	2;												//У��λʧ�ܷ���2
		}
		status = rfid_rx_buf[4];									//��ȡ���ذ�״̬
		if(status == 0)												//�ж��Ƿ���ȷ�Ķ�����������0Ϊ�ɹ���1Ϊʧ��
		{
			return 0;
		}
	}
	return 1;														//����ʧ�ܷ���1
}
