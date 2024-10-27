#include "arm_task.h"
#include "arm_solver.h"
#include "pid.h"
#include "usbd_cdc_if.h"
#include "unitree_a1.h"

//#define printf myprintf

float targ_pos = 0;
float ramp_targ_pos = 0;
float arm_slow_start_k = 0.0008;
float real_pos = 0;
float arm_zero_pose = 1.14955831f;	//�ض����������ת����ֱλ�õġ����ԽǶȡ�


uint8_t direction = 0; //1��2��
extern unitree_ctrl_t unitree_Data;
extern serial_servo_t servo_Data;
extern struct arm_solver solver;
extern uint8_t TX_shijue_mode;
extern uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM * 2];

arm_ctrl_point point;

bool_t arm_safe = 1;//��е�۵��Ա���λ
bool_t unitree_init_flag = 0;//����ϵ��ʼ����־λ
bool_t arm_ctrl_signal = 0;

uint16_t servo_start_flag = 99;//ÿ����100�����߶�������ź�ִ��һ��(��1sִ��һ��)

bool_t A1_die_flag = 0;

uint8_t TX_shijue_flag = 99;

void arm_task(void const * argument)
{
	osDelay(10);
	Arm_Init();
	osDelay(10);
	
	while(1)
	{
		//���Ա���
		while(arm_safe)
		{
			unitree_torque_ctrl(&unitree_Data, 0);
			ramp_targ_pos = unitree_Data.unitree_recv.Pos;
			osDelay(10);
		}
		
		if(A1_die_flag == 1)
		{
			unitree_torque_ctrl(&unitree_Data, 0);
			unitree_Uart_RE_Init(unitree_rx_buf[0], unitree_rx_buf[1], Unitree_RX_BUF_NUM);
			__HAL_UART_CLEAR_PEFLAG(&UNITREE_MOTOR_HUART);
			A1_die_flag = 0;
		}
		
		//��ʼת�������ֵλ��
		if(!unitree_init_flag)
		{
			targ_pos = arm_zero_pose;
			if(fabs(unitree_Data.unitree_recv.Pos-targ_pos) <= 0.005f)
			{
				targ_pos = 0;
				real_pos = 0;
				ramp_targ_pos = 0;
				unitree_Data.zero_pose = arm_zero_pose;
				unitree_init_flag = 1;
				arm_ctrl_signal = 1;
			}
		}
		
/**********��е�۽�������ó���***********/
		else
		{
			servo_start_flag++;
			if(servo_start_flag >= servo_Data.serial_servo_Time)	//100ms����һ�ζ�������źţ��鿴����Ƕ�
			{
				if(arm_ctrl_signal)	//����ͬʱ����
				{
					arm_solve(point.total_angle, point.x, point.y);
					servo_arm_move(solver.a1, solver.a2);	//���ƶ��ת���ƶ��Ƕ�
					targ_pos = -solver.a0;					//���õ���Ƕ�
				}
				else
				{
					getServosAngle(3,1,2,3);
				}
				servo_start_flag = 0;
				
				//����һ�µط������Ӿ�ģʽ
				CDC_Transmit_FS(&TX_shijue_mode,1);
				
			}
/***********************************************/	
		}
		
//		if(TX_shijue_flag >= 100)
//		{
//			CDC_Transmit_FS(&TX_shijue_mode,1);
//			TX_shijue_flag = 0;
//		}
//		TX_shijue_flag ++;
		
		//��۵��λ�ÿ���
		ramp_function(&ramp_targ_pos, targ_pos, arm_slow_start_k); 
		unitree_pos_pid_ctrl(ramp_targ_pos);
		unitree_save_check();
		real_pos = unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose;
		osDelay(1);
	}
}
