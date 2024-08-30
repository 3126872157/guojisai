#include "task.h"

//宇树
extern uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];//双缓冲数组

extern unitree_Data_1 unitree_Data;
//幻尔
extern uart_servo_Data_1 uart_servo_Data;
extern uint8_t LobotTxBuf[128];  //发送缓存
extern uint8_t LobotRxBuf[16];   //接收缓存 
//开环舵机
extern servo_data_1 servo_data;
void mechanical_arm_task_Init(void)
{
	//初始化宇树
	Unitree_Usart6_Init(Unitree_rx6_buf[0],Unitree_rx6_buf[1],Unitree_RX_BUF_NUM);
	//初始化幻尔
	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);
	//开环舵机初始化
	servo_Init();
}

void mechanical_arm_task_send(void)
{
	//宇树发送数据
	ModifyData(&unitree_Data.unitree_send,0,unitree_Data.unitree_MODE,unitree_Data.unitree_Torque,unitree_Data.unitree_W,unitree_Data.unitree_POS,unitree_Data.unitree_KP,unitree_Data.unitree_KW);
	UnitreeSend(&unitree_Data.unitree_send);
	//幻尔发送数据
	
	//开环舵机发送数据
	//servo_Move(speed_1,speed_2);
}

void mechanical_arm_task_receive(void)
{
	//宇树接收数据
	ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
	//幻尔接收数据
	
}

void mechanical_arm_task_updata(void)//计算后得到的数据
{
	
}

