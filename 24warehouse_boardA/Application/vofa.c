#include "VOFA.h"


float send_data[10]={0.0f};

uint8_t send_buff[80];

void float_to_u8_v(float* datain,uint8_t* dataout)
{
    uint8_t farray[4];
    *(float*)farray=*datain;
    dataout[3]=farray[3];
    dataout[2]=farray[2];
    dataout[1]=farray[1];
    dataout[0]=farray[0];
}


//参数：发送的数据指针，可填多个int my_vofa_printf(const char *format,...)
void my_vofa_printf(uint8_t len)
{
	
	uint8_t tail[4]={0x00,0x00,0x80,0x7f};//vofa中justfloat帧尾
	
	for(uint8_t i=0;i<len;i++)
	{
		uint8_t f_t_u[4];
		float_to_u8_v(&send_data[i], f_t_u);
		for(int j = 4*i;j < 4*(i+1);j++)
			send_buff[j]=f_t_u[j%4];
	}
	
	send_buff[len*4] = tail[0];
	send_buff[len*4+1] = tail[1];
	send_buff[len*4+2] = tail[2];
	send_buff[len*4+3] = tail[3];

	HAL_UART_Transmit_DMA(&huart8,send_buff,len*4+4);
	
//	HAL_UART_Transmit(&huart6,temps,length*4+4,0xFF);
}


