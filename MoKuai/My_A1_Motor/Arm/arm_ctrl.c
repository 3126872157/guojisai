#include "main.h"
//#include "tim.h"
#include "arm_ctrl.h"
//#include "uart_servo.h"
#include "unitree_a1.h"

extern uint8_t motor_rx_temp;

motr_ctr_t unitree_Data;
//uart_servo_Data_1 uart_servo_Data;
//servo_data_1 servo_data;

void Arm_Init(void)
{
//	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);	//幻尔初始化
	Unitree_Usart6_Init(Unitree_rx6_buf[0],Unitree_rx6_buf[1],Unitree_RX_BUF_NUM);//宇树初始化
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);  //开启PWM通道
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);  //开启PWM通道
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);  //开启PWM通道
//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);  //开启PWM通道
	//宇树初始角度
	uint8_t check = 0;
	uint8_t a[2] = {0};
	if(check == 0)
	{
		ModifyData(&unitree_Data.unitree_send,0,10,0,0,0,0);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_data_rx,&motor_rx_temp);
		a[1] = a[0];
		a[0] = unitree_Data.unitree_data_rx.Pos;
		unitree_Data.zero_pose = unitree_Data.unitree_data_rx.Pos;
		if(unitree_Data.zero_pose != 0)
		{
			if((a[0] - a[1])<0.05||(a[1] - a[0])<0.05)
			{					
				unitree_Data.zero_pose = unitree_Data.unitree_data_rx.Pos;
				check = 1;
			}
		}
	}
}
