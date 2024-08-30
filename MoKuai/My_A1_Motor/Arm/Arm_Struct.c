#include "main.h"
//#include "tim.h"
#include "Arm_Struct.h"
//#include "servo.h"
//#include "uart_servo.h"
#include "unitree_a1.h"

unitree_Data_1 unitree_Data;
uart_servo_Data_1 uart_servo_Data;
servo_data_1 servo_data;

void Arm_Init(void)
{
//	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);	//�ö���ʼ��
	Unitree_Usart6_Init(Unitree_rx6_buf[0],Unitree_rx6_buf[1],Unitree_RX_BUF_NUM);//������ʼ��
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //����PWMͨ��
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //����PWMͨ��
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //����PWMͨ��
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //����PWMͨ��
	//������ʼ�Ƕ�
	uint8_t check = 0;
	uint8_t a[2] = {0};
	if(check == 0)
	{
		ModifyData(&unitree_Data.unitree_send,0,10,0,0,0,0,0);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
		a[1] = a[0];
		a[0] = unitree_Data.unitree_data_rx.Pos;
		unitree_Data.pos0 = unitree_Data.unitree_data_rx.Pos;
		if(unitree_Data.pos0 != 0)
		{
			if((a[0] - a[1])<0.05||(a[1] - a[0])<0.05)
			{					
				unitree_Data.pos0 = unitree_Data.unitree_data_rx.Pos;
				check = 1;
			}
		}
	}
}