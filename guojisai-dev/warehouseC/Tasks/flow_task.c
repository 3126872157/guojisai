#include "flow_task.h"
#include "chassis_behaviour.h"
#include "bodanpan.h"
#include "Bodanpan_Task.h"
#include "arm_control_task.h"
#include "arm_ctrl.h"
#include "math.h"
#include "arm_solver.h"

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

//ȡ����벦���̺�ı�־λ
extern bool_t a_new_ball_in;

//���ģʽ����
extern uint8_t lizhuang_ball_num;//��׮�е��˼�����
uint8_t ball_error = 0;

//�Ҷȴ��������ݣ�0Ϊ��ɫ��ɫ��1Ϊ��ɫ��Ŀǰ��Ӳ������ֻ��һ·����gray_data[0]
extern uint8_t gray_data[2];

//��е�ۿ����йز���
extern uint8_t arm_control_mode;
extern bool_t arm_ctrl_signal;
extern uint8_t arm_current_step;
extern bool_t nan_error;

//�Ӿ��йز���
extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 0;//��ʱ�ȹص���ʹ�ö�������
float shijue_k = -1;
uint8_t shijue_error = 0;
extern uint8_t TX_shijue_mode;
uint8_t licang_current_line;//�ֿ������¼

float shijue_suoqiu_tolerance = 10;//�Ӿ���������ֵ
float shijue_suoqiu_tolerance2 = 3;//�Ӿ�������������ֵ

float shijue_suozhang_tolerance = 10;//�Ӿ���������ֵ(���ڱ���ǰ�ľ�׼���Ķ�λ)
float obstacle_x_tol = 10;//���ڴ���Ⱥ��ƹ����е������ϰ�
float QR_x_tol = 10;//������ά������ֵ
float obstacle_distance_tol = 250;//���ϰ���ǰ���پ���ͣ��

uint8_t QR_code[4];//0��1��2�ֱ��¼������QRֵ�����ĸ��ò���
uint8_t QR_num;
uint8_t QR_doing[3] = {0};
uint8_t QR_PutBall_num = 2;

//��ֹ��ʶ��ʮ��
float bizhang_distance = 0;
float bizhang_distance_total = 165;

bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

/*******************���̿���ģʽ����********************
0����е��
1������ƽ�Ƽ�ת��λ�û�
2��������ϻҶȴ�������ǰ�ߣ���⵽����ͣ
3����������Ӿ������ƶ�����⵽��ͣ
4����е�۽���ƽ̨ȡ��
5����е����׮ȡ��
6������ƽ��λ�û�����Ӿ�����
7������ƽ��λ�û�����Ӿ����Ϻ�ǰ��
8���Ӿ��������ϣ����е���
9��ת�̻���е�۾�λ
10�����̺�������ά��ͣ
11�����ֵ����е�ۼ���
66������״̬������debug����
********************************************************/

//��е������ʼ�ı�־λ
bool_t modeN_task_start = 0;
bool_t mode9_task_start = 0;
bool_t bogan_zhunbei_flag = 1;
bool_t bogan_jiqiu_flag = 0;
uint8_t zhuanpanji_ball_num = 0;
uint8_t bogan_delay = 100;
bool_t zhuanpanji_finish_flag = 0;

//���Ʊ����б���ʽ����
//����ģʽ		para1		para2		para3		�����ж����ֵ			�����˶�ģʽ
//0				
//1				x			y       	gyro   		distance_tol��gyro_tol	CHASSIS_MOVE_AND_ROTATE
//2				x_speed		y_speed											CHASSIS_V
//3				x_speed		y_speed											CHASSIS_V
//4																			CHASSIS_MOVE_AND_ROTATE
//5

const uint8_t non = 0;


float test_v_max = 5;

TargetPoints targ_point[] = {
		//��׮
	
///*0*/	{67,		 1000,	 		0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
///*1*/	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
///*2*/	{67,		 0,	 		100,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
///*3*/	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
///*4*/	{67,		 -100,	 		0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
///*5*/	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
///*6*/	{67,		 0,	 		-100,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},
///*26*/	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��


		//��㵽��׮
/*0*/	{1,		 0,	 		35,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//�ߵ���׮ǰ
/*1*/	{1,		 190,	 	0,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//�ߵ���׮ǰ
		//��׮����
/*2*/	{3,		 0,		  	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ�����
/*3*/	{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����
/*4*/	{3,		 0,		  	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ�����
		{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
/*5*/	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����
		{22,	 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��׮����Ƿ������
		//������׮������ƽ̨����
/*6*/	{1,		 -20,	 	0,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//��׮��ɣ�����20
/*7*/	{1,		 0,		  	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//˳ʱ��ת90
/*8*/	{1,		 75,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
/*9*/	{1,		 0,	 		20,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
/*10*/	{2,		 100,	     0,			-90,		5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ

		//����ƽ̨
/*11*/	{3,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*3*/	{21,	 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
/*12*/	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ��һ��
	
/*13*/	{3,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*3*/	{21,	 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
/*14*/	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ�ڶ���

		//����ƽ̨��Բ�̻�����
/*15*/	{1,		 -20,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//����ƽ̨��ɣ�����20
/*16*/	{1,		 0,	 		0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//��ʱ����ת180���Ա��Ӿ�����
/*17*/	{1,		 0,		  	-140,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����ǰ
/*18*/	{6,		 0,		  	-200,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//����ƽ������
/*19*/	{7,		 200,		 0,			0,			5,				CHASSIS_MOVE_AND_ROTATE},//���Ϻ���ǰ��������С�ڸ���ֵ�������϶���
/*20*/	{8,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*21*/	{1,		 0,	 		40,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶���
/*22*/	{1,		 90,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶���
/*23*/	{1,		 0,	 		-40,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶����������ƫһ�㣬��һ�㲹������ʶ��ʮ�ֵ�������Ҫ����һ��
/*24*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
	
		//Բ�̻�
/*25*/	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��е�۾�λ,���˲���
	
/*26*/	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
		//Բ�̻����ֿ����
/*27*/	{1,		 -5,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����5
/*28*/	{1,		 0,	 		150,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����150cm����ൽ����

		//���ֵ��⣨�ظ����Σ�
/*29*/	{10,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ�ƽ�Ƽ�¼3����ά���Ӧ��λ�� �������ƶ�
//		{3,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*30*/	{2,		 100,	     0,			90,			non,			CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
//		{19,	 0,		  	100,			0,		5,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����һ�㣬�����ݶ�ά�����ּ�¼��ǰ����
/*33*/	{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
/*34*/	{13,	 0,	 		-18,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
/*35*/	{2,		 100,	     0,			90,			non,			CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		
/*37*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά�룬��ֹ͵��ע�����
		{16,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ�����2��ά��
//		{12,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ����ض���ά��
//		{3,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*35*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
//		{19,	 0,			100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*41*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
		{13,	 0,	 		-18,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
/*44*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		
/*46*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά��
		{14,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ�����3��ά��
//		{12,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ����ض���ά��
//		{3,		 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
/*48*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
//		{19,	 0,		  	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*50*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
		{13,	 0,	 		-18,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
/*53*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
/*54*/	{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*55*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����


		//���ַ���
/*56*/	{14,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����3��ά��
/*57*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{18,	 1,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 2,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 3,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*61*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�����ά��
		
/*62*/	{16,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����2��ά��
/*63*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
/*64*/	{18,	 1,/*�к�*/	2,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*65*/	{18,	 2,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*66*/	{18,	 3,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�����ά��
		
		{17,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����1��ά��
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{18,	 1,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 2,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 3,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*73*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����

		//������ȥ����ƽ̨
/*74*/	{1,		 0,	 		0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//��ת180
		{1,		 0,	 		-20,		-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 120,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 0,	 		30,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
/*78*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
/*79*/	{20,	 1,/*�к�*/	4,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		
		{66,		 non,	 	non,		non,		non,			CHASSIS_V}
	
	
};

//��ǰ����λ�����
uint8_t currentTargIndex = 0;
//��������־
uint8_t isFinished = 0;


bool_t flag;

uint16_t test_pos = 250;
uint16_t test_pos_2 = 600;
void flow_task(void const * argument)
{
	while(1)
	{
//		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, test_pos);//����
//		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, test_pos_2);//����
//		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, test_pos);
		//�ж����в����Ƿ�����
		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
		{
			//������ǰĿ��
			TargetPoints target = targ_point[currentTargIndex];
			
			//�����˶�
			if(target.mode == 1)
			{
				//���õ����˶�Ŀ��
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.gyro_set = target.para3;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				//�ж����
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					
					//�����ã�����Ҫ���Ӿ���Ҫ666
//					if(currentTargIndex == 33 || currentTargIndex == 43 || currentTargIndex == 53)
//					{
//						osDelay(100);
//						for(int i  = 0;i < 3;i++)
//						{
//							//��¼��ǰ���������
//							if(shijue_data.QR_code == QR_code[i])
//							{
//								QR_doing[i] = 1;
//								break;
//							}
//						}
//					}
					
					//��̼�����
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//��⵽���߾�ͣ
			else if(target.mode == 2)
			{
				//���õ����˶�Ŀ��
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				gray_sensor_read();
				
				//�߼������ƣ���������������������������������
				if(gray_data[1] == 0)
				{
					if(currentTargIndex != 24)
					{
						chassis_code_reset_flag = 1;
						modeN_task_start = 0;
						currentTargIndex ++;
					}
					else
					{
						if((bizhang_distance + 90.0f + chassis_move.x) > bizhang_distance_total)
						{
							chassis_code_reset_flag = 1;
							V_mode_x_speed = 0;
							currentTargIndex ++;
						}
					}
				}
			}
			//�����Ӿ�����
			else if(target.mode == 3)
			{
				if(modeN_task_start == 0)
				{
					osDelay(100);
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
								
				//�������
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//�����ұߣ�������
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//��666������ʾʶ�𲻵������ո�������
				else if(fabs(shijue_data.ball_x - 666) < 2)
					chassis_move.y_set = target.para2;

				if(chassis_move.y_set < 0)
					shi_jue_x_pianzhi = -shi_jue_x_pianzhi;//�������ҷ������ƫ��
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					//������Լӽ����ж������������������ɨ�򣬻���ֱ����currentTargetIndexָ����һ������
					//���Զ࿪һ��last currentTargIndex
					currentTargIndex ++;
				}
			}
			//����ƽ̨
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					if(shijue_data.ball_y < 0)/*����Ϊ���һ��*/
					{
						arm_control_mode = 1;
						modeN_task_start = 1;
					}
					
					else if(shijue_data.ball_y < 8 && shijue_data.ball_y > 0/*����Ϊ�м�һ��*/)
					{
						arm_control_mode = 2;
						modeN_task_start = 1;
					}
					else if(shijue_data.ball_y > 9/*����Ϊ���һ��*/)
					{
						arm_control_mode = 3;
						modeN_task_start = 1;
					}
				}
				if(arm_control_mode == 0)
				{
					modeN_task_start = 0;

					currentTargIndex ++;
				}
			}	
			
			//��е����׮ȡ��
			else if(target.mode == 5)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					arm_control_mode = 10;
//					osDelay(100);
//					if(nan_error == 1)//����Ҫ��ôд
					modeN_task_start = 1;
				}
				if(arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//�Ӿ�����
			else if(target.mode == 6)
			{
				//���õ����˶�Ŀ��
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				if(fabs(shijue_data.obstacle_x) < obstacle_x_tol)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//���ϣ�ǰ�������ϰ���
			else if(target.mode == 7)
			{
				//���õ����˶�Ŀ��
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				if(fabs(shijue_data.obstacle_distance) < obstacle_distance_tol)
				{
					//��̼�����		
					bizhang_distance += chassis_move.x;
					modeN_task_start = 0;
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
				
			}
			
			//���ϣ������ϰ���
			else if(target.mode == 8)
			{
				
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
								
				//�ϰ������
				if(shijue_data.obstacle_x < 0)
					chassis_move.y_set = target.para2;
				//�ϰ����ұߣ�������
				else if(shijue_data.obstacle_x > 0)
					chassis_move.y_set = -target.para2;
								
				if(fabs(shijue_data.obstacle_x) < shijue_suozhang_tolerance)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//ת�̻���е�۲���
			else if(target.mode == 9)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(mode9_task_start == 0)
				{
					arm_control_mode = 11;
					mode9_task_start = 1;
					osDelay(5000);
				}
				
				if(bogan_jiqiu_flag == 0 && shijue_data.ball_distance == 1)
				{
					bogan_control(1);
					
					osDelay(bogan_delay);
					
					bogan_jiqiu_flag = 1;
					bogan_zhunbei_flag = 0;
					zhuanpanji_ball_num++;
					if(zhuanpanji_ball_num >= 2)
						a_new_ball_in = 1;
					
				}
				
				if(bogan_zhunbei_flag == 0 && shijue_data.ball_distance == 0)
				{
					bogan_control(2);
					bogan_zhunbei_flag = 1;
					bogan_jiqiu_flag = 0;
				}				
				
				if(zhuanpanji_ball_num == 6)
				{
					osDelay(2000);//�����ӳ�һ�£���ֹ���һ�λ���û�ɹ�����е�۾�̧������
					zhuanpanji_finish_flag = 1;
					a_new_ball_in = 1;
				}
				if(arm_control_mode == 0)
				{
					mode9_task_start = 0;
					currentTargIndex ++;
				}

			}
			
			//�����Ӿ����ά��
			else if(target.mode == 10)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				//��ά������Ҹ��ϴζ����Ĳ�һ��ʱ����β����ʼ��
				if((shijue_data.QR_code == 1 || shijue_data.QR_code == 2 || shijue_data.QR_code == 3) &&
						(shijue_data.QR_code != QR_code[0] && shijue_data.QR_code != QR_code[1] && shijue_data.QR_code != QR_code[2]))
				{
					QR_code[2-QR_num] = shijue_data.QR_code;
					QR_num++;
				}
				
				if(QR_num == 3 && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					QR_num = 0;
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//�������
			else if(target.mode == 11)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					if(shijue_data.ball_y < -5)//���һ�㣬ԭ��ֵΪ-10
					{
						arm_control_mode = 7;
						modeN_task_start = 1;
						licang_current_line = 3;
					}
					//�м��
					else if(shijue_data.ball_y > 5 && shijue_data.ball_y < 25)//ԭ��ֵΪ10��20
					{
						arm_control_mode = 8;
						modeN_task_start = 1;
						licang_current_line = 2;
					}
					else if(fabs(shijue_data.ball_y - 666) < 2)//���һ��
					{
						arm_control_mode = 9;
						modeN_task_start = 1;
						licang_current_line = 1;
					}
					else//���һ��
					{
						arm_control_mode = 9;
						modeN_task_start = 1;
						licang_current_line = 1;
					}
				}
				if(arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}	
			
			//�Ӿ����ض���ά��
			else if(target.mode == 12)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				uint8_t i = 0;
				for(i = 0;i < 3; ++i)
				{
					if(QR_doing[i] == 0)
					{
						break;
					}
				}
				if(shijue_data.QR_code == QR_code[i] && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//����3�Ķ�ά��
			else if(target.mode == 14)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				//�����һ����ά�룬��3
				if(shijue_data.QR_code == QR_code[2] && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//��е�����ֵ������
			else if(target.mode == 15)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					arm_control_mode = licang_current_line + 11;
					modeN_task_start = 1;
				}
				if(arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//����2��ά�룬����
			else if(target.mode == 16)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				if(shijue_data.QR_code == QR_code[1] && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//����1��ά�룬����
			else if(target.mode == 17)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				if(shijue_data.QR_code == QR_code[0] && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//���ַ���
			else if(target.mode == 18)
			{
				chassis_behaviour_mode = target.chassis_mode;
				//���û�ҵ�����˵�������ȡʧ��ֱ���������η���
				bool_t ball_find_error;
				if(modeN_task_start == 0)
				{
					if(target.para1 == 1)
					{
						ball_find_error = bodanpan_find_ball(target.para1, QR_code[QR_PutBall_num]);
						QR_PutBall_num --;
					}		
					else
						ball_find_error = bodanpan_find_ball(target.para1, target.para2);
					
					if(ball_find_error == 1)
					{
						modeN_task_start = 0;
						currentTargIndex ++;
					}
					else
					{
						arm_control_mode = target.para1 + 3;
						modeN_task_start = 1;
					}
				}
				
				if(ball_find_error == 0 && arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
				
			}
			
			else if(target.mode == 19)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
				
				if(fabs(shijue_data.ball_x - 666) < 2)
				{
					chassis_move.y_set = 0;
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
					
				//�������
				else if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//�����ұߣ�������
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//��666������ʾʶ�𲻵�������

				if(chassis_move.y_set < 0)
					shi_jue_x_pianzhi = -shi_jue_x_pianzhi;//�������ҷ������ƫ��
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//���������
			else if(target.mode == 20)
			{
				chassis_behaviour_mode = target.chassis_mode;
				
				//���û�ҵ�����˵�������ȡʧ��ֱ���������η���
				bool_t ball_find_error;
				if(modeN_task_start == 0)
				{
					ball_find_error = bodanpan_find_ball(target.para1, target.para2);
					if(ball_find_error == 1)
					{
						modeN_task_start = 0;
						currentTargIndex ++;
					}
					else
					{
						arm_control_mode = 15;
						modeN_task_start = 1;
					}
				}
				
				if(ball_find_error == 0 && arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
				
			}
			
			else if(target.mode == 21)
			{
				if(modeN_task_start == 0)
				{
					chassis_behaviour_mode = target.chassis_mode;
					chassis_move.x_set = target.para1;
					chassis_move.y_set = target.para2;
					chassis_move.vx_max_speed = target.V_max;
					chassis_move.vx_min_speed = -target.V_max;
					chassis_move.vy_max_speed = target.V_max;
					chassis_move.vy_min_speed = -target.V_max;
					modeN_task_start = 1;
				}
								
				//�������
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//�����ұߣ�������
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//��666������ʾʶ�𲻵������ո�������
				else if(fabs(shijue_data.ball_x - 666) < 2)
					chassis_move.y_set = target.para2;

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance2)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 22)
			{
				if(lizhuang_ball_num != 2 && ball_error != 1)
				{
					ball_error ++;
					currentTargIndex -= 3;
				}
				else
				{
					ball_error = 0;
					currentTargIndex ++;
				}
					
			}
			
			//������
			else if(target.mode == 66)
			{
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.vx_set = V_mode_x_speed;
				chassis_move.vy_set = V_mode_y_speed;
				if(take_a_ball)
				{
					bodanpan_find_ball(ball_x,ball_y);
					take_a_ball = 0;
				}
				
			}
			
			else if(target.mode == 67)
			{
				//���õ����˶�Ŀ��
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				chassis_move.vx_max_speed = test_v_max;
				chassis_move.vx_min_speed = -test_v_max;
				chassis_move.vy_max_speed = test_v_max;
				chassis_move.vy_min_speed = -test_v_max;
				
				//�ж����
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}

			
			if(currentTargIndex <= 16) 
				TX_shijue_mode = 0;
			else if(currentTargIndex <= 24)
				TX_shijue_mode = 1;
			else if(currentTargIndex <= 25)
				TX_shijue_mode = 2;
			else if(currentTargIndex <= 80)
				TX_shijue_mode = 3;
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
