#include "flow_task.h"
#include "chassis_behaviour.h"

#define distance_tol 2
#define gyro_tol 0.2f
//���Ƶ����˶�
extern chassis_move_t chassis_move;
extern chassis_mode_e chassis_behaviour_mode;
extern bool_t chassis_code_reset_flag;

//CHASSIS_Vģʽ��������������ٶ�
extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

//�Ҷȴ��������ݣ�0Ϊ��ɫ��ɫ��1Ϊ��ɫ��Ŀǰ��Ӳ������ֻ��һ·����gray_data[1]
extern uint8_t gray_data[2];


extern shijue_Data shijue_data;
float shijue_k = -1;

//���Ʊ����б���ʽ����
//���̻��߻�е��   x�����ߵľ���   y�����ߵľ���   ת��ĽǶ�  �����ж����ֵ   �����˶�ģʽ
//1Ϊ����			��λcm        	 ÿ������      ��λcm      				 3���ߺ�ת
//0Ϊ��е��
//2Ϊ������ϻҶȴ�������ǰ�ߣ���⵽����ͣ
//3Ϊ��������Ӿ������ƶ�����⵽��ͣ
//4Ϊ����״̬�������ֶ�����
TargetPoints targ_point[] = {
	{1,		 190,	 -80,	0,		distance_tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		  0,	-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 0,	      0,	0,		distance_tol,		CHASSIS_V},
	{3,		 0,		  0,	0,		distance_tol,		CHASSIS_V},
	{4,		 0,	     40,	0,		gyro_tol,			CHASSIS_V},
	{1,		-80,	 0,		0,		distance_tol,		CHASSIS_V},
	{1,		 0,		-80,	0,		distance_tol,		CHASSIS_V},
	{3,		 0,		0,		0,		distance_tol,		CHASSIS_V}
};

//��ǰ����λ�����
uint8_t currentTargIndex = 0;
//��������־
uint8_t isFinished = 0;

bool_t flag;

void flow_task(void const * argument)
{
	while(1)
	{
		//�ж����в����Ƿ�����
		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
		{
			//������ǰĿ��
			TargetPoints target = targ_point[currentTargIndex];
			
			//Ŀǰ������
			if(target.mode == 1)
			{
				//���õ����˶�Ŀ��
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.x;
				chassis_move.y_set = target.y;
				chassis_move.gyro_set = target.gyro;
				//�ж����
				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.gyro) < gyro_tol)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_x_speed = 5;
				
				if(gray_data[1] == 0)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 3)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_y_speed = 5;
				
				if(flag/*�Ӿ�����*/)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
			}
		}
		
		else
		{
			isFinished ++;
			//ɾ����ǰ����
			vTaskDelete(NULL);
		}
		osDelay(1);
	}
}
