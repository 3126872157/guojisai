#include "flow_task.h"
#include "main.h"
#include "chassis_behaviour.h"

//���Ƶ����˶�
extern chassis_move_t chassis_move;
#define tol 1
extern bool_t chassis_code_reset_flag;

extern shijue_Data shijue_data;
float shijue_k = 100;;

//���Ʊ����б���ʽ����
//���̻��߻�е��   x�����ߵľ���   y�����ߵľ���   �����ж����ֵ   �����˶�ģʽ
//1Ϊ����0Ϊ��е��    ��λcm         ÿ������          ��λcm       3���ߺ�ת
TargetPoints targ_point[] = {
	{1,		 80,	 0,		tol,		3},
	{1,		 0,		80,		tol,		3},
	{1,		-80,	 0,		tol,		3},
	{1,		 0,		-80,	tol,		3}
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
				chassis_move.chassis_mode = target.chassis_mode;
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
				chassis_move.chassis_mode = target.chassis_mode;
				chassis_move.x_set = 0;
				chassis_move.y_set = shijue_data.ball_x/shijue_k;
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
