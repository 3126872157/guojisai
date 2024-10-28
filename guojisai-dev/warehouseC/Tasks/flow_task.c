#include "flow_task.h"
#include "chassis_task.h"
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
int8_t shi_jue_x_pianzhi = 5;//��ʱ�ȹص���ʹ�ö�������
float shijue_k = -1;
uint8_t shijue_error = 0;
extern uint8_t TX_shijue_mode;
uint8_t licang_current_line;//�ֿ������¼

float shijue_suoqiu_tolerance = 20;//�Ӿ�һ����������ֵmode3
float shijue_suoqiu_tolerance2 = 5;//�Ӿ�������������ֵmode21
float shijue_suoqiu_tolerance3 = 2;//�Ӿ�������������ֵmode13

float shijue_suozhang_tolerance = 10;//�Ӿ���������ֵ(���ڱ���ǰ�ľ�׼���Ķ�λ)
float obstacle_x_tol = 50;//���ں��ƹ����е������ϰ�
float QR_x_tol = 50;//������ά������ֵ
float obstacle_distance_tol = 250;//���ϰ���ǰ���پ���ͣ��

uint8_t QR_code[4];//0��1��2�ֱ��¼������QRֵ�����ĸ��ò���
uint8_t QR_num;
uint8_t QR_doing[3] = {0};
uint8_t QR_PutBall_num = 2;

//��ֹ��ʶ��ʮ��
float bizhang_distance = 0;
float bizhang_distance_total = 130;

bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

//ת�̻���ʱ����
extern uint16_t Time_s;
uint16_t start_time;//ת�̻���ʼʱ��

//�ؼ�
bool_t x_home_finish = 0;
bool_t y_home_finish = 0;


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
uint8_t bogan_delay = 150;
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
	
		//��㵽��׮
/*0*/	{1,		 182,	 		35,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//�ߵ���׮ǰ

		//��׮����
/*1*/	{3,		 0,		  	-100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ�����
		{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{13,	 0,		  	-100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����

/*5*/	{3,		 0,		  	-100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ�����
		{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{13,	 0,		  	-100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����
		//������׮������ƽ̨����
/*9*/	{1,		 -20,	 	0,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//��׮��ɣ�����20
		{1,		 0,		  	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//˳ʱ��ת90
		{1,		 75,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
		{1,		 0,	 		20,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
		{2,		 100,	     0,			-90,		5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ

		//����ƽ̨
/*14*/	{3,		 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
		{21,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{13,	 0,		  	100,		0,			1,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ��һ��
	
/*18*/	{3,		 0,		  	100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ���������
		{21,	 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{13,	 0,		  	100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//�����Ӿ���������
		{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ�ڶ���

		//����ƽ̨��Բ�̻�����
/*22*/	{1,		 -20,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//����ƽ̨��ɣ�����20
		{1,		 0,	 		0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//��ʱ����ת180���Ա��Ӿ�����
		{1,		 0,		  	-120,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����ǰ
		{6,		 0,		  	-200,		0,			10,				CHASSIS_MOVE_AND_ROTATE},//����ƽ������
		{7,		 200,		 0,			0,			5,				CHASSIS_MOVE_AND_ROTATE},//���Ϻ���ǰ��������С�ڸ���ֵ�������϶���
		{1,		 0,	 		40,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶���
		{1,		 90,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶���
		{1,		 0,	 		-45,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���϶����������ƫһ�㣬��һ�㲹������ʶ��ʮ�ֵ�������Ҫ����һ��
/*30*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
	
		//Բ�̻�
/*31*/	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��е�۾�λ,���˲���
	
		//Բ�̻����ֿ����
/*32*/	{1,		 -5,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����5
/*33*/	{1,		 0,	 		150,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����150cm����ൽ����

		//���ֵ��⣨�ظ����Σ�
/*34*/	{10,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�Ӿ�ƽ�Ƽ�¼3����ά���Ӧ��λ�� �������ƶ�
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{19,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����һ�㣬�����ݶ�ά�����ּ�¼��ǰ����
/*39*/	{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
		{1,	 	 0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		
/*43*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά�룬��ֹ͵��ע�����
		{16,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ�����2��ά��
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{19,	 0,			100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
		{1,	 	0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
/*51*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		
/*53*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά��
		{14,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//���ƣ�����3��ά��
/*55*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{19,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//���ָİ��Ӿ���������
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*58*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұߵĶ�ά�룬��λ��
		{1,	 	 0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����

/*93*/	{66,		non,	 	non,		non,		non,			CHASSIS_V},

		//���ַ���
/*64*/	{14,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����3��ά��
/*65*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{18,	 1,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 2,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 3,/*�к�*/	3,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*69*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�����ά��
		
/*70*/	{16,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����2��ά��
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{18,	 1,/*�к�*/	2,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 2,/*�к�*/	2,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 3,/*�к�*/	2,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//���ˣ�����ά��
		
/*76*/	{17,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//�����ұ������ߣ�����1��ά��
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
		{18,	 1,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 2,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{18,	 3,/*�к�*/	1,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*81*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����

		//������ȥ����ƽ̨
/*82*/	{1,		 0,	 		0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//��ת180
		{1,		 0,	 		-20,		-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 120,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 0,	 		50,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
/*86*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//ǰ���Ҷ�ʶ����ߺ�ͣ
/*87*/	{20,	 1,/*�к�*/	4,/*�к�*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//����

		//�ؼ�
		{1,		 -20,	 	0,			-90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		  0,	 	0,			0,			    50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 -200,	 	55,			0,				50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{22,	   -50,	 	50,			0,				5,				CHASSIS_MOVE_AND_ROTATE},//����ؼ�
		{23,	   10,	 	-10,		0,				5,				CHASSIS_MOVE_AND_ROTATE},//����ؼ�
		{1,		  -3,	 	3,			0,				5,				CHASSIS_MOVE_AND_ROTATE},
/*93*/	{66,	non,	 	non,		non,		non,			CHASSIS_V}
	
	
};

//��ǰ����λ�����
/******************************************************************************/
uint8_t currentTargIndex = 23;//ע�⣡����
/******************************************************************************/

//��������־
uint8_t isFinished = 0;


bool_t flag;

uint16_t test_pos = 250;
uint16_t test_pos_2 = 600;
void flow_task(void const * argument)
{
	while(1)
	{
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, test_pos);//����
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, test_pos_2);//����
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, test_pos);
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
					if(currentTargIndex != 30)
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
							modeN_task_start = 0;
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
					if(shijue_data.ball_y < 5)/*����Ϊ���һ��*/
					{
						arm_control_mode = 1;
						modeN_task_start = 1;
					}
					
					else if(shijue_data.ball_y < 15 && shijue_data.ball_y > 5/*����Ϊ�м�һ��*/)
					{
						arm_control_mode = 2;
						modeN_task_start = 1;
					}
					else if(shijue_data.ball_y > 15/*����Ϊ���һ��*/)
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
					osDelay(4000);
					start_time = Time_s;
				}
				
				if(bogan_jiqiu_flag == 0 && shijue_data.ball_distance == 1)
				{
					bogan_control(1);
					osDelay(bogan_delay);
					bogan_control(2);
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
				
				if(/*zhuanpanji_ball_num == 6*/Time_s - start_time > 120)
				{
					bogan_control(2);
//					osDelay(500);//�����ӳ�һ�£���ֹ���һ�λ���û�ɹ�����е�۾�̧������
					zhuanpanji_finish_flag = 1;
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
				uint8_t ball_find_error = 0;
				if(modeN_task_start == 0)
				{
					if(target.para1 == 1)
					{
						ball_find_error = bodanpan_find_ball(target.para1, QR_code[QR_PutBall_num]);
						QR_PutBall_num --;
					}		
					else
						ball_find_error = bodanpan_find_ball(target.para1, target.para2);
					
//					if(ball_find_error == 1)
//					{
//						modeN_task_start = 0;
//						currentTargIndex ++;
//					}
//					else
//					{
						osDelay(500);
						arm_control_mode = target.para1 + 3;
						modeN_task_start = 1;
//					}
				}
				
				if(/*ball_find_error == 0 &&*/ arm_control_mode == 0)
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

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance2)
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
				uint8_t ball_find_error;
				if(modeN_task_start == 0)
				{
					ball_find_error = bodanpan_find_ball(target.para1, target.para2);
//					if(ball_find_error == 1)
//					{
//						modeN_task_start = 0;
//						currentTargIndex ++;
//					}
//					else
//					{
						osDelay(500);
						arm_control_mode = 15;
						modeN_task_start = 1;
//					}
				}
				
				if(/*ball_find_error == 0 &&*/ arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
				
			}
			
			//�Ӿ����κ�������
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
			
			//�Ӿ����κ�������
			else if(target.mode == 13)
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

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance3)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//�ؼ�
			else if(target.mode == 22)
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
				
				if(gray_data[1] == 0 && x_home_finish == 0)//��ǰ�Ҷ�
				{
					x_home_finish = 1;
					chassis_move.x_set = 0;
					chassis_move.x = 0;
				}
				if(gray_data[0] == 1 && y_home_finish == 0)//���һҶ�
				{
					y_home_finish = 1;
					chassis_move.y_set = 0;
					chassis_move.y = 0;
				}
				if(x_home_finish == 1 && y_home_finish == 1)
				{
					chassis_code_reset_flag = 1;
					x_home_finish = 0;
					y_home_finish = 0;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 23)
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
				
				if(gray_data[1] == 1 && x_home_finish == 0)//��ǰ�Ҷ�
				{
					x_home_finish = 1;
					chassis_move.x_set = 0;
					chassis_move.x = 0;
				}
				if(gray_data[0] == 0 && y_home_finish == 0)//���һҶ�
				{
					y_home_finish = 1;
					chassis_move.y_set = 0;
					chassis_move.y = 0;
				}
				if(x_home_finish == 1 && y_home_finish == 1)
				{
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			
//			else if(target.mode == 22)
//			{
//				if(lizhuang_ball_num != 2 && ball_error != 1)
//				{
//					ball_error ++;
//					currentTargIndex -= 3;
//				}
//				else
//				{
//					ball_error = 0;
//					currentTargIndex ++;
//				}
//					
//			}
			
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

			
			if(currentTargIndex <= 21) 
				TX_shijue_mode = 0;
			else if(currentTargIndex <= 29)
				TX_shijue_mode = 1;
			else if(currentTargIndex == 31)
				TX_shijue_mode = 2;
			else if(currentTargIndex <= 100)
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
