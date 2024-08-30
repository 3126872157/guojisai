/*
M3650A-HA协议

数据包格式:
  0			1		   2 			 3 		   4 				5-20 			  21
包类型	  包长度		返回命令			地址 	  状态 			   数据信息 			校验和
 0x04 	   0x16       0x03 			0x20 	0x00:成功 	      16字节数据 		   X

数据输出：
0x04 0x16 0x03 0x20 0x00 16字节数据 X校验和

校验和：
X=前21字节依次按位异或最后再取反

建议：
调试最好配合从客服那拿的资料，阅读手册和是用读写器测试软件
软件可以用来设置波特率，写入块数据等等
*/


#include "stm32f4xx.h"                  // Device header
#include "usart.h"

#define RFID_RX_BUF_MAX_LEN 30

//命令
uint8_t Cmd_Read_Id[8] = {0x01,0x08,0xa1,0x20,0x00,0x00,0x00,0x00};
uint8_t Cmd_Read_Block[8]	= {0x01,0x08,0xa3,0x20,0x00,0x00,0x00,0x00};
uint8_t Cmd_Write_Block[23] = {0x01,0x17,0xa4,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t Cmd_Set_Work_Mode[8] = {0x03,0x08,0xC1,0x20,0x03,0x01,0x00,0x00};
uint8_t Cmd_Set_AutoRead[8] = {0x03,0x08,0xC8,0x20,0x01,0x00,0x00,0x00};

//串口接收相关
uint8_t buf[1] = {0};
uint8_t rfid_rx_buf[30] = {0};
uint8_t rfid_rx_flag = 0;		//指示接受完毕完整数据帧
uint8_t rfid_error = 0;			//指示串口接收函数出现重大失误
//ic卡串口接收相关
uint8_t ic_read_start = 0;
uint8_t ic_buf[22] = {0};
uint8_t ic_buf_size = 0;
uint8_t X = 0;

//初始化
void rfid_init(void)
{
	
	HAL_UART_Receive_IT(&huart1, buf, 1);
}

//串口接收函数，测试模式的鲁棒性较差！！？？
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static uint8_t rfid_rx_count = 0;
	if(huart->Instance == USART1)
	{
		
		/********************测试模式*******************/
		rfid_rx_buf[rfid_rx_count] = buf[0];
		rfid_rx_count ++;
		if(rfid_rx_count >= RFID_RX_BUF_MAX_LEN)
		{
			rfid_rx_count = 0;
			rfid_error = 1;		//大事不妙啦
		}
		if(rfid_rx_buf[1] == rfid_rx_count)	
		{
			rfid_rx_count = 0;	//如果对齐出现错误就完蛋了
			rfid_rx_flag = 1;
		}
		/********************测试模式*******************/
		
		
		/********************上赛模式*******************/
		if(ic_read_start == 1)
		{
			if(ic_buf_size == 1 && buf[0] != 0x16)		//第二帧对不上
			{
				ic_read_start = 0;
				ic_buf_size = -1;
			}
			ic_buf[ic_buf_size++] = buf[0];
			
			if(ic_buf_size == 22)						//接收完毕
			{
				ic_read_start = 0;
				ic_buf_size = 0;						//计数归零
				for(uint8_t i = 0;i < 21 ; i++)
				{
					X^=ic_buf[i];
				}
				X=~X;
				if(X == ic_buf[21])						//校验成功
				{
					uint8_t flag = 1;
					for(uint8_t i=0;i<9;i++)			//判断数据是否已经存过
					{
						if(ic_buf[15]==bodanpan.date[i])
						{
							flag = 0;
						}
					}
					if(flag)							//存ic数据
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
		
		if(ic_read_start == 0 && buf[0] == 0x04)//检测到帧头
		{
			ic_read_start = 1;
			ic_buf[ic_buf_size++] = buf[0];
		}
		/********************上赛模式*******************/
		HAL_UART_Receive_IT(&huart1, buf, 1);
	}
}

//Rx模式校验检测
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
		return 	1;	//校验OK
	else 
		return 	0;	//校验ERROR
}

//Tx模式CRC验证计算
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

//读取ID，包括2位卡类型，4位卡号
uint8_t rfid_ReadId(unsigned char *IDout)
{
	uint8_t status;
	uint8_t i;
	Cmd_Read_Id[5] = 0x01;											//开启蜂鸣器提示
	TxCheckSum(Cmd_Read_Id, Cmd_Read_Id[1]);						//计算校验和
	HAL_UART_Transmit(&huart1, Cmd_Read_Id, Cmd_Read_Id[1], 100);	//发送读卡号ID命令	 
	//等待模块返回数据，大于1.5MS
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf, rfid_rx_buf[1]);			//对接收到的数据校验
		if(status != 1)  											//判断校验和是否正确
		{
			return 2;												//校验位失败返回2
		}
		status = rfid_rx_buf[4];
		if(status != 1)												//判断是否正确的读到卡
		{
		 	return 3;												//不知怎么失败的返回3
		}
		if((rfid_rx_buf[0] == 0x01)&&(rfid_rx_buf[2] == 0xa1))		//判断是否为读卡号返回的数据包
		{
			for(i=0;i<6;i++)										//获取卡号ID，6字节		 
			{
				IDout[i] = rfid_rx_buf[i+5];						//从数组的第5个字节开始为卡号，长度为6字节
			}
			return 0;		 										//成功返回0
		}
 	} 
	return 1;														//串口读取失败返回1
}

//指令读
uint8_t rfid_ReadFromBlock(uint8_t *dataout,uint8_t block)
{
	uint8_t status;
	uint8_t i;
	Cmd_Read_Block[4] = block;
	Cmd_Read_Block[5] = 0x01;										//开启蜂鸣器提示
	TxCheckSum(Cmd_Read_Block, Cmd_Read_Block[1]);					//数据校验
	HAL_UART_Transmit(&huart1, Cmd_Read_Block, Cmd_Read_Block[1], 100);	//发送读卡号ID命令	
	//等待模块返回数据，大于1.5ms
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf,rfid_rx_buf[1]);			//对接收到的数据校验
		if(status != 1)		 										//判断校验和是否正确
		{
			return 	2;												//校验位失败返回2
		}
		status = rfid_rx_buf[4];									//获取返回包状态
		if(status != 1)												//判断是否正确的读到卡
		{
			return 3;
		}
		if((rfid_rx_buf[0] == 0x01)&&(rfid_rx_buf[2] == 0xa3))		//判断是否为读块数据返回的数据包
		{
			for(i=0;i<16;i++)										//获取块数据，16字节	，一个数据块的大小为16字节	 
			{
				dataout[i] = rfid_rx_buf[i+5];						//从数组的第5个字节开始为数据，长度为16字节
			}
			return 0;		 										//成功返回0
		}
	}
	return 1;														//失败返回1
}

//设置自动读卡方式：1为一直读，0为读一次
uint8_t rfid_Set_AutoRead(uint8_t autoread)						
{
	uint8_t status;
	Cmd_Set_AutoRead[4] = autoread;
	TxCheckSum(rfid_rx_buf, rfid_rx_buf[1]);						//数据校验
	HAL_UART_Transmit(&huart1, Cmd_Set_AutoRead, Cmd_Set_AutoRead[1], 100);	//发送读卡号ID命令	
	//等待模块返回数据，大于1.5ms
	HAL_Delay(2);
 	if(rfid_rx_flag == 1)
 	{	
		rfid_rx_flag = 0;
		status = RxCheckSum(rfid_rx_buf,rfid_rx_buf[1]);			//对接收到的数据校验
		if(status != 1)		 										//判断校验和是否正确
		{
			return 	2;												//校验位失败返回2
		}
		status = rfid_rx_buf[4];									//获取返回包状态
		if(status == 0)												//判断是否正确的读到卡，返回0为成功，1为失败
		{
			return 0;
		}
	}
	return 1;														//串口失败返回1
}
