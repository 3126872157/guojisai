#include "flow_task.h"

//���Ƶ����˶�
extern chassis_move_t chassis_move;

//���Ʊ����б���ʽ����
//���̻��߻�е��   x�����ߵľ���   y�����ߵľ���   �����ж����ֵ   �����˶�ģʽ
//1Ϊ����0Ϊ��е��    ��λcm         ÿ������          ��λcm       3���ߺ�ת
TargetPoints targ_point[] = {
	{1,		 20,	 0,		5,		3},
	{1,		 0,		20,		5,		3},
	{1,		-20,	 0,		5,		3},
	{1,		 0,		-20,	5,		3}
};

//��ǰ����λ�����
uint8_t currentTargIndex = 0;
//��������־
uint8_t isFinished = 0;

//void vFlowTask(void* pvParameters)
//{
//	while(1)
//	{
//		//�ж����в����Ƿ�����
//		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
//		{
//			//������ǰĿ��
//			TargetPoints target = targ_point[currentTargIndex];
//			
//			//Ŀǰ������
//			if(target.mode == 1)
//			{
//				//���õ����˶�Ŀ��
//				chassis_move.chassis_mode = target.chassis_mode;
//				chassis_move.x_set = target.x;
//				chassis_move.y_set = target.y;
//				
//				//�ж����
//				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
//				if(distance < target.tolerance)
//				{
//					currentTargIndex ++;
//				}
//			}
//		}
//		else
//		{
//			isFinished ++;
//			//ɾ����ǰ����
//			vTaskDelete(NULL);
//		}
//	}
//}