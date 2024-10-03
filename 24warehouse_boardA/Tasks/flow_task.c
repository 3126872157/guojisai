#include "flow_task.h"
#include "chassis_behaviour.h"

#define tol 2

//���Ƶ����˶�
extern chassis_move_t chassis_move;
extern chassis_mode_e chassis_behaviour_mode;
extern bool_t chassis_code_reset_flag;

//CHASSIS_Vģʽ��������������ٶ�
extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

extern shijue_Data shijue_data;
float shijue_k = -1;

//���Ʊ����б���ʽ����
//���̻��߻�е��   x�����ߵľ���   y�����ߵľ���   �����ж����ֵ   �����˶�ģʽ
//1Ϊ����0Ϊ��е��    ��λcm         ÿ������          ��λcm       3���ߺ�ת
TargetPoints targ_point[] = {
	{1,		 80,	 0,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		80,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		-80,	 0,		tol,		CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		-80,	tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 80,	0,	    tol,		CHASSIS_V},
	{2,		 0,		80,		tol,		CHASSIS_V},
	{2,		-80,	 0,		tol,		CHASSIS_V},
	{2,		 0,		-80,	tol,		CHASSIS_V},
	{3,		 0,		0,		tol,		CHASSIS_V}
};

//��ǰ����λ�����
uint8_t currentTargIndex = 0;
//��������־
uint8_t isFinished = 0;

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
				
				//�ж����
				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
				if(distance < target.tolerance)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;

				if(target.x > 0) V_mode_x_speed = 5;
				else if(target.x < 0) V_mode_x_speed = -5;
				else V_mode_x_speed = 0;
					
				if(target.y > 0) V_mode_y_speed = 5;
				else if(target.y < 0) V_mode_y_speed = -5;
				else V_mode_y_speed = 0;

				
//				chassis_move.x_set = 0;
//				chassis_move.y_set = shijue_data.ball_x/shijue_k;	
				
				if(fabs(chassis_move.x-target.x) < target.tolerance)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex++;
				}
				
			}
			else if(target.mode == 3)//�ٶȵ���ģʽ
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
