#include "arm_control_task.h"
#include "arm_ctrl.h"
#include "arm_task.h"

extern serial_servo_t servo_Data;
extern bool_t arm_ctrl_signal;

extern arm_ctrl_point point;


uint8_t arm_control_mode = 0;
uint8_t arm_current_step = 0;

uint16_t extra_time = 1000;

void set_bodanpan_pos(void)
{
	point.x = 275;
	point.y = -35;
	point.total_angle = 175;
}

void set_normal_pos(void)
{
	point.x = 300;
	point.y = 300;
	point.total_angle = 90;
}

void set_jieti_middle_pos(void)
{
	point.x = 400;
	point.y = 400;
	point.total_angle = 75;
}

void arm_control_task(void const * argument)
{
	while(1)
	{
		switch(arm_control_mode)
		{
			//��ͨģʽ
			case 0:
				set_normal_pos();
				break;
			//�ý���ƽ̨��ߵĽ��ݵ���
			case 1:
				switch(arm_current_step)
				{
					case 0:
						arm_ctrl_signal = 1;
						claw_control(2);	//claw�м�λ��
						point.x = 485;			//��߽��ݵ�����
						point.y = 245;
						point.total_angle = 90;
						arm_current_step ++;
						break;
					case 1:
						claw_control(0);	//claw��ȡ
						arm_current_step ++;
						//����Լ��Ƿ�е�����ж�
						break;
					case 2:					//����
						set_jieti_middle_pos();
						arm_current_step ++;
						break;
					case 3:					//����
						set_normal_pos();
						arm_current_step ++;
						break;
					case 4:
						set_bodanpan_pos();
						arm_current_step ++;
						break;
					case 5:
						claw_control(1);
						arm_current_step ++;
						break;
					case 6:
						set_normal_pos();
						arm_current_step ++;
						break;
					case 7:
						arm_control_mode = 0;
						arm_current_step = 0;
						arm_ctrl_signal = 0;
						break;
				}	
		}
		//ȷ����е�ۺͶ���������
		osDelay((uint32_t)servo_Data.serial_servo_Time + (uint32_t)extra_time);
	}
}
