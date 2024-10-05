#include "flow_task.h"
#include "chassis_behaviour.h"
#include "bodanpan.h"
#include "Bodanpan_Task.h"
#include "arm_control_task.h"

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

//��е�ۿ����йز���
extern uint8_t arm_control_mode;


extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;
float shijue_k = -1;


bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

/*******************���̿���ģʽ����********************
0����е��
1������ƽ�Ƽ�ת��λ�û�
2��������ϻҶȴ�������ǰ�ߣ���⵽����ͣ
3������ƽ̨��������Ӿ������ƶ�����⵽��ͣ
4����е�۽���ƽ̨ȡ��
5������״̬������debug����
********************************************************/

bool_t mode4_task_start = 0;

//���Ʊ����б���ʽ����
//����ģʽ		para1		para2		para3		�����ж����ֵ			�����˶�ģʽ
//0				
//1				x			y       	gyro   		distance_tol��gyro_tol	CHASSIS_MOVE_AND_ROTATE
//2				x_speed		y_speed											CHASSIS_V
//3				x_speed		y_speed											CHASSIS_V
//4																			CHASSIS_MOVE_AND_ROTATE
//5

const uint8_t non = 0;

TargetPoints targ_point[] = {
	{1,		 190,	 	-80,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
	{1,		 0,		  	0,			-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},
	{2,		 5,	      	0,			0,			non,			CHASSIS_V},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},
	{4,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},
	{5,		 non,		non,		non,		non,			CHASSIS_V}
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
		
			if(target.mode == 1)
			{
				//���õ����˶�Ŀ��
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				//�ж����
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
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
				V_mode_x_speed = target.para1;
				
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
				V_mode_y_speed = target.para2;
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < 1)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
					currentTargIndex ++;
				}
			}
			
			if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(mode4_task_start == 0)
				{
					if(fabs(shijue_data.ball_distance)/*����Ϊ���һ��*/)
					{
						arm_control_mode = 1;
						mode4_task_start = 1;
					}	
					
					else if(fabs(shijue_data.ball_distance)/*����Ϊ�м�һ��*/)
					{
						arm_control_mode = 2;
						mode4_task_start = 1;
					}
					else if(fabs(shijue_data.ball_distance)/*����Ϊ���һ��*/)
					{
						arm_control_mode = 3;
						mode4_task_start = 1;
					}
				}
				if(arm_control_mode == 0)
					currentTargIndex ++;
			}	
			
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(take_a_ball)
					bodanpan_find_ball(ball_x,ball_y);
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
