#include "task.h"

//����
extern uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];//˫��������

extern unitree_Data_1 unitree_Data;
//�ö�
extern uart_servo_Data_1 uart_servo_Data;
extern uint8_t LobotTxBuf[128];  //���ͻ���
extern uint8_t LobotRxBuf[16];   //���ջ��� 
//�������
extern servo_data_1 servo_data;
void mechanical_arm_task_Init(void)
{
	//��ʼ������
	Unitree_Usart6_Init(Unitree_rx6_buf[0],Unitree_rx6_buf[1],Unitree_RX_BUF_NUM);
	//��ʼ���ö�
	uart_servo_UART5_Init(LobotRxBuf,uart_servo_BUFFER_SIZE);
	//���������ʼ��
	servo_Init();
}

void mechanical_arm_task_send(void)
{
	//������������
	ModifyData(&unitree_Data.unitree_send,0,unitree_Data.unitree_MODE,unitree_Data.unitree_Torque,unitree_Data.unitree_W,unitree_Data.unitree_POS,unitree_Data.unitree_KP,unitree_Data.unitree_KW);
	UnitreeSend(&unitree_Data.unitree_send);
	//�ö���������
	
	//���������������
	//servo_Move(speed_1,speed_2);
}

void mechanical_arm_task_receive(void)
{
	//������������
	ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
	//�ö���������
	
}

void mechanical_arm_task_updata(void)//�����õ�������
{
	
}

