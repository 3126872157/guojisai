#include "flow_task.h"
#include "chassis_behaviour.h"
#include "bodanpan.h"
#include "Bodanpan_Task.h"
#include "arm_control_task.h"
#include "arm_ctrl.h"

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

//�Ҷȴ��������ݣ�0Ϊ��ɫ��ɫ��1Ϊ��ɫ��Ŀǰ��Ӳ������ֻ��һ·����gray_data[0]
extern uint8_t gray_data[2];

//��е�ۿ����йز���
extern uint8_t arm_control_mode;
extern bool_t arm_ctrl_signal;
extern uint8_t arm_current_step;
extern uint16_t claw_middle_pos;

//�Ӿ��йز���
extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;
float shijue_k = -1;
uint8_t shijue_error = 0;
float shijue_suoqiu_tolerance = 5;//�Ӿ���������ֵ
float shijue_suozhang_tolerance = 5;//�Ӿ���������ֵ(���ڱ���ǰ�ľ�׼���Ķ�λ)
float obstacle_x_tol = 10;//���ڴ���Ⱥ��ƹ����е������ϰ�
float QR_x_tol = 10;//������ά������ֵ
float obstacle_distance_tol = 250;//���ϰ���ǰ���پ���ͣ��
extern uint8_t TX_shijue_mode;
uint8_t licang_current_line;//�ֿ������¼
uint8_t QR_code[4];//0��1��2�ֱ��¼������QRֵ�����ĸ��ò���
uint8_t QR_num;
uint8_t QR_doing[3] = {0};
uint8_t QR_doing_num = 0;

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
bool_t mode4_task_start = 0;
bool_t mode5_task_start = 0;
bool_t mode9_task_start = 0;
bool_t mode11_task_start = 0;
bool_t bogan_zhunbei_flag = 1;
bool_t bogan_jiqiu_flag = 0;
uint8_t zhuanpanji_ball_num = 0;

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
		//��׮
/*0*/	{1,		 0,	 		35,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//�ߵ���׮ǰ
/*1*/	{1,		 195,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//�ߵ���׮ǰ
/*2*/	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},				 //�����Ӿ�����
/*3*/	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����
/*4*/	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},				 //�����Ӿ�����
/*5*/	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//���ģʽ����

		//������׮������ƽ̨����
/*6*/	{1,		 -20,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//��׮��ɣ�����20
/*7*/	{1,		 0,		  	0,			-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//˳ʱ��ת90
/*8*/	{1,		 60,	 	0,			-90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
/*9*/	{1,		 0,	 		8,			-90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//������������ƽ̨�Ҳ�
/*10*/	{2,		 5,	      	0,			0,			non,			CHASSIS_V},				 //ǰ���Ҷ�ʶ����ߺ�ͣ

		//����ƽ̨
/*11*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //�Ӿ���������
/*12*/	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ��һ��
	
/*13*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //�Ӿ���������
/*14*/	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ�ڶ���
	
/*15*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //�Ӿ���������
/*16*/	{4,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��ȡ������
	
	
//	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
		//����ƽ̨��Բ�̻�����
/*17*/	{1,		 -20,	 	0,			-90,		distance_tol,	CHASSIS_MOVE_AND_ROTATE},//����ƽ̨��ɣ�����20
/*18*/	{1,		 0,	 		0,			90,			gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//��ʱ����ת180���Ա��Ӿ�����
/*19*/	{1,		 0,		  	-80,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//����ǰ
/*20*/	{6,		 0,		  	-5,			0,			non,			CHASSIS_V},//����ƽ������
/*21*/	{7,		 5,		 	0,			0,			non,			CHASSIS_V},//���Ϻ���ǰ��������С�ڸ���ֵ�������϶���
/*22*/	{8,		 0,		  	5,			0,			non,			CHASSIS_V},				 //�Ӿ���������
/*23*/	{1,		 0,	 		40,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���϶���
/*24*/	{1,		 90,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���϶���
/*25*/	{1,		 0,	 		-40,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���϶����������ƫһ�㣬��һ�㲹������ʶ��ʮ�ֵ�������Ҫ����һ��
/*26*/	{2,		 10,	     0,			0,			non,			CHASSIS_V},//ǰ���Ҷ�ʶ����ߺ�ͣ
	
		//Բ�̻�
/*27*/	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//��е�۾�λ,���˲���
	
/*28*/	{8,		 non,	 	non,		non,		non,			CHASSIS_V},//�������ԣ�����Ҫ��ɾ��
		//Բ�̻����ֿ����
/*29*/	{1,		 -15,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//����15
/*30*/	{1,		 0,	 		140,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//����130cm����ൽ����

		//���ֵ��⣨�ظ����Σ�
/*31*/	{10,	 0,	      	5,			0,			non,			CHASSIS_V},//�Ӿ�ƽ�Ƽ�¼3����ά���Ӧ��λ�� �������ƶ�
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//�Ӿ���������
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//����ǰ���Ҷ�ʶ����
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
		{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����һ�㣬�����ݶ�ά�����ּ�¼��ǰ����
/*36*/	{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//�����ұߵĶ�ά�룬��λ��
/*37*/	{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//ǰ���Ҷ�ʶ����ߺ�ͣ��Ȼ�����
		//����
/*38*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά�룬��ֹ͵��ע�����
		{12,	0,	 		5,			90,			distance_tol,	CHASSIS_V},//���ƣ����ض���ά��
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//�Ӿ���������
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//����ǰ���Ҷ�ʶ����
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*43*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//�����ұߵĶ�ά�룬��λ��
		{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//ǰ���Ҷ�ʶ����ߺ�ͣ��Ȼ�����
		//����
/*47*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ�������Ͷ�ά��
		{12,	0,	 		5,			90,			distance_tol,	CHASSIS_V},//���ƣ����ض���ά��
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//�Ӿ���������
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//����ǰ���Ҷ�ʶ����
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//����
/*50*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ���ʶ���ά�룬�����ݶ�ά�����ּ�¼��ǰ����
		{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//�����ұߵĶ�ά�룬��λ��
		{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//���Ƶ�����λ�����Ե���һ������ģʽ�����ٶȱ���
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//ǰ���Ҷ�ʶ����ߺ�ͣ��Ȼ�����
		//����

/*34*/
/*35*/
/*36*/
/*37*/

		//���ַ���
/*40*/	{3,		 0,	      	5,			0,			non,			CHASSIS_V},//�����ұ������ߣ��������ά��
/*41*/	{2,		 5,	     	0,			0,			non,			CHASSIS_V},//ǰ���Ҷ�ʶ����ߺ�ͣ����������
/*42*/	{1,		 -15,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//���ˣ�����ά�룬�ظ�
		
	

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
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				//�ж����
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					
					//�����ã�����Ҫ���Ӿ���Ҫ666
					if(currentTargIndex >= 29 && currentTargIndex <= 60)
					{
						for(int i  = 0;i<3;i++)
						{
							//��¼��ǰ���������
							if(shijue_data.QR_code == QR_code[i])
							{
								QR_doing[i] = 1;
							}
						}
					}
					
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//��⵽���߾�ͣ
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_x_speed = target.para1;
				
				//�߼������ƣ���������������������������������
				if(gray_data[0] == 0)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					currentTargIndex ++;
				}
			}
			//�����Ӿ�����
			else if(target.mode == 3)
			{
				chassis_behaviour_mode = target.chassis_mode;
				TX_shijue_mode = 0;
//				V_mode_x_speed = target.para1;
//				V_mode_y_speed = target.para2;
				
				//�������
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = target.para2;
				}
				//�����ұߣ�������
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = -target.para2;
				}
				
				//��666������ʾʶ�𲻵������ո�������
				else if(fabs(shijue_data.ball_x - 666) < 2)
				{	
					V_mode_y_speed = target.para2;
					shijue_error ++;
				}

				if(V_mode_y_speed < 0)
				{	
					//�������ҷ������ƫ��
					shi_jue_x_pianzhi = -shi_jue_x_pianzhi;
				}
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					V_mode_y_speed = 0;
					
					//������Լӽ����ж������������������ɨ�򣬻���ֱ����currentTargetIndexָ����һ������
					//���Զ࿪һ��last currentTargIndex
					currentTargIndex ++;
				}
			}
			//����ƽ̨
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				TX_shijue_mode = 0;
				if(mode4_task_start == 0)
				{
					if(shijue_data.ball_y < 0)/*����Ϊ���һ��*/
					{
						arm_control_mode = 1;
						mode4_task_start = 1;
					}
					
					else if(shijue_data.ball_y < 8 && shijue_data.ball_y > 0/*����Ϊ�м�һ��*/)
					{
						arm_control_mode = 2;
						claw_middle_pos = 450;
						mode4_task_start = 1;
					}
					else if(shijue_data.ball_y > 9/*����Ϊ���һ��*/)
					{
						arm_control_mode = 3;
						claw_middle_pos = 450;
						mode4_task_start = 1;
					}
				}
				if(arm_control_mode == 0)
				{
					a_new_ball_in = 1;
					mode4_task_start = 0;
					claw_middle_pos = 470;
					currentTargIndex ++;
				}
			}	
			
			//��е����׮ȡ��
			else if(target.mode == 5)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(mode5_task_start == 0)
				{
					arm_control_mode = 10;
					mode5_task_start = 1;
				}
				if(arm_control_mode == 0)
				{
					a_new_ball_in = 1;
					mode5_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//�Ӿ�����
			else if(target.mode == 6)
			{
				//���õ����˶�Ŀ��
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				if(fabs(shijue_data.obstacle_x) < obstacle_x_tol)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//���ϣ������ϰ���
			else if(target.mode == 7)
			{
				//���õ����˶�Ŀ��
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				if(fabs(shijue_data.obstacle_distance) < obstacle_distance_tol)
				{
					//��̼�����			
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
				
			}
			
			//���ϣ������ϰ���
			else if(target.mode == 8)
			{
				TX_shijue_mode = 2;			
				chassis_behaviour_mode = target.chassis_mode;
				//�ϰ������
				if(shijue_data.obstacle_x < 0)
				{
					V_mode_y_speed = target.para2;
				}
				//�ϰ����ұ�
				else if(shijue_data.obstacle_x > 0)
				{
					V_mode_y_speed = -target.para2;
				}
								
				if(fabs(shijue_data.obstacle_x) < shijue_suozhang_tolerance)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
					currentTargIndex ++;
				}
			}
			
			//ת�̻���е�۲���
			else if(target.mode == 9)
			{
				TX_shijue_mode = 1;
				chassis_behaviour_mode = target.chassis_mode;
				if(mode9_task_start == 0)
				{
					arm_control_mode = 10;
					mode9_task_start = 1;
					osDelay(1000);
				}
				
				if(bogan_jiqiu_flag == 0 && shijue_data.ball_distance == 1)
				{
					bogan_control(1);
					bogan_jiqiu_flag = 1;
					bogan_zhunbei_flag = 0;
					zhuanpanji_ball_num++;
				}
				
				if(bogan_zhunbei_flag == 0 && shijue_data.ball_distance == 0)
				{
					bogan_control(2);
					bogan_zhunbei_flag = 1;
					bogan_jiqiu_flag = 0;
				}
				
				
				/*******����a_new_ball_in��֪��զ�ӣ�������*******/
				
				
				if(zhuanpanji_ball_num == 5)
				{
					osDelay(500);//�����ӳ�һ�£���ֹ���һ�λ���û�ɹ�����е�۾�̧������
					arm_control_mode = 0;
					arm_current_step = 0;
					arm_ctrl_signal = 0;
					currentTargIndex ++;
				}

			}
			
			//�����Ӿ����ά��
			else if(target.mode == 10)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				
				//��ά������Ҹ��ϴζ����Ĳ�һ��ʱ����β����ʼ��
				if((shijue_data.QR_code == 1 || shijue_data.QR_code == 2 || shijue_data.QR_code == 3) &&
						(shijue_data.QR_code != QR_code[0] && shijue_data.QR_code != QR_code[1] && shijue_data.QR_code != QR_code[2]))
				{
					QR_code[2-QR_num] = shijue_data.QR_code;
					QR_num++;
				}
				
				if(QR_num == 3 && fabs(shijue_data.QR_x) < 10)
				{
					QR_num = 0;
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//�������
			else if(target.mode == 11)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;
				if(mode11_task_start == 0)
				{
					if(shijue_data.ball_y < -10)//���һ��
					{
						arm_control_mode = 7;
						mode11_task_start = 1;
						licang_current_line = 3;
					}
					
					else if(shijue_data.ball_y > 10 && shijue_data.ball_y < 20)
					{
						arm_control_mode = 8;
						mode11_task_start = 1;
						licang_current_line = 2;
					}
					else if(shijue_data.ball_y == 666)//���һ��
					{
						arm_control_mode = 9;
						mode11_task_start = 1;
						licang_current_line = 1;
					}
				}
				if(arm_control_mode == 0)
				{
					mode11_task_start = 0;
					currentTargIndex ++;
				}
			}	
			
			//�Ӿ����ض���ά��
			else if(target.mode == 12)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				
				uint8_t i = 0;
				for(i = 0;i<3;++i)
				{
					if(QR_doing[i] == 0)
					{
						break;
					}
				}
				
				if(shijue_data.QR_code == QR_code[i] && shijue_data.QR_x < 10)
				{
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//���ٵ��ƶ��������ͣ�£�������
			else if(target.mode == 13)
			{
				chassis_behaviour_mode = target.chassis_mode;
				V_mode_x_speed = 0;
				V_mode_y_speed = -5;
				
				//�ж����
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					//��̼�����
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//�����Ķ�ά��
			else if(target.mode == 14)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				
				//�����һ����ά�룬��3
				if(shijue_data.QR_code == QR_code[2] && shijue_data.QR_x < 10)
				{
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}

			//������
			else if(target.mode == 66)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(take_a_ball)
				{
					bodanpan_find_ball(ball_x,ball_y);
					take_a_ball = 0;
				}
				chassis_move.x_set = 0;
				chassis_move.y_set = 0;
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
