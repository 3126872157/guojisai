#include "arm_control_task.h"
#include "arm_ctrl.h"
#include "arm_task.h"
#include "Bodanpan_Task.h"
#include "math.h"

#define D2R 0.0174532925f

extern bool_t a_new_ball_in;

extern serial_servo_t servo_Data;
extern bool_t arm_ctrl_signal;

extern arm_ctrl_point point;

extern shijue_Data shijue_data;
uint8_t arm_shijue_error = 0;
float piancha = 75;//����ͷ����צ�ľ���ƫ��


//�Ӹ�����0,1,2
float jie_ti_ping_tai[3] = {260, 200, 150};	//����ƽ̨y�����˵ڶ����м��yֵ
//�Ӹ�����0,1,2
float li_cang[3] = {370, 250, 110};//{355, 235, 115}
float jieti_x = 500;//����ƽ̨x��490
float lizhuang_x =  250.0f;
float lizhuang_angle = 20.65f * D2R;

uint8_t arm_control_mode = 0;
uint8_t arm_current_step = 0;

uint16_t extra_time = 1000;

void set_bodanpan_pos(void)
{
	point.x = 275;
	point.y = -25;
	point.total_angle = 180;
}

void set_normal_pos(void)
{
	point.x = 300;
	point.y = 300;
	point.total_angle = 110;
	tulun_control(0);
	bogan_control(0);
}

void set_jieti_middle_pos(void)
{
	point.x = 400;
	point.y = 400;
	point.total_angle = 75;
}

//extra_time 0��arm_slow_start_k 0.001
extern float arm_slow_start_k;//�ؽڵ���������



void jie_ti_ping_tai_take_dingceng(void)//arm_control_mode = 1
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				claw_control(470);
				arm_current_step ++;
				break;
			case 1:
//				uint8_t num = 0;
//				float distance_total = 0;
//				while(num < 100)
//				{
//					distance_total += shijue_data.ball_y;
//					num ++;
//					osDelay(1);
//				}
//				if(fabs(shijue_data.ball_x - 666) > 2)
//				{
//					point.x += 10.0f + (distance_total/100.0f - piancha) * cosf(lizhuang_angle) - shijue_data.ball_y * sinf(lizhuang_angle);
//				}
				extra_time = 500;
			
				point.x = jieti_x;		//���ݵ�����
				point.y = jie_ti_ping_tai[0];
				point.total_angle = 100;
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				claw_control(400);	//claw��ȡ
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 3:					//����
				extra_time = 0;
			
				set_jieti_middle_pos();
				arm_current_step ++;
				break;
//			case 5:					//��׼������
//				set_bodanpan_pos();
//				arm_current_step ++;
//				break;
			case 4:					//��׼����
				extra_time = 0;
			
				point.x = 360;
				point.y = 200;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:					//����
				extra_time = 100;
			
				claw_control(470);	
				arm_current_step ++;
				break;
			case 6:					//�ع�
				extra_time = 0;
			
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 7:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				extra_time = 1000;
			
				a_new_ball_in = 1;
				break;
		}
}


void jie_ti_ping_tai_take(uint8_t jie_ti_num)//arm_control_mode = 2 , 3
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 200;
			
				arm_ctrl_signal = 1;
//				uint8_t num = 0;
//				float distance_total = 0;
//				while(num < 100)
//				{
//					distance_total += shijue_data.ball_y;
//					num ++;
//					osDelay(1);
//				}
//				if(fabs(shijue_data.ball_x - 666) > 2)
//				{
//					point.x += 10.0f + (distance_total/100.0f - piancha) * cosf(lizhuang_angle) - shijue_data.ball_y * sinf(lizhuang_angle);
//				}
			
				point.x = jieti_x;		//���ݵ�����
				point.y = jie_ti_ping_tai[jie_ti_num] + 50;
				claw_control(400);	//�����������أ��ȹرռ�צ,400
				point.total_angle = 110;//����ƽ̨ĩ�˽Ƕ�,100
				arm_current_step ++;
				break;
				
			case 1:
				extra_time = 0;
			
				claw_control(490);	//�ſ���צ
				arm_current_step ++;
				break;
				
			case 2:
				extra_time = 100;
			
				point.y = jie_ti_ping_tai[jie_ti_num];
				arm_current_step ++;
				break;
			case 3:
				extra_time = 0;
			
				claw_control(400);	//claw��ȡ��410
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 4:					//����
				extra_time = 0;
			
				set_jieti_middle_pos();
				arm_current_step ++;
				break;
			case 5:					//����
				extra_time = 0;
			
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 6:					//��׼����
				extra_time = 0;
			
				point.x = 360;
				point.y = 200;
				point.total_angle = 110;
				arm_current_step ++;
				break;
//			case 5:					//��׼������
//				set_bodanpan_pos();
//				arm_current_step ++;
//				break;
			case 7:					//����
				extra_time = 0;
			
				claw_control(470);	
				arm_current_step ++;
				break;
			case 8:					//�ع�
				extra_time = 0;
			
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 9:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				extra_time = 1000;
			
				a_new_ball_in = 1;
				break;
		}
}

void li_cang_take(uint8_t li_cang_num)//arm_control_mode = 7 8
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
				arm_slow_start_k = 0.001;
				arm_ctrl_signal = 1;
				point.x = 380;		//���ֵ�����
				point.y = li_cang[li_cang_num];
				point.total_angle = 100;
				claw_control(470);
				arm_current_step ++;
				break;
			case 1:
				extra_time = 0;
			
				claw_control(470);	
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				point.x = 480;
				arm_current_step ++;
				break;
			case 3:		
				extra_time = 0;
			
				claw_control(540);	
				point.y = li_cang[li_cang_num] - 60;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				claw_control(400);	
				arm_current_step ++;
				break;
			case 5:
				extra_time = 0;
			
				point.y = li_cang[li_cang_num] - 30;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				point.x = 380;
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 8:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				arm_slow_start_k = 0.0008;
				break;
		}
}

void li_cang_take_diceng(void)//arm_control_mode = 9
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 500;
			
				arm_ctrl_signal = 1;
				huadao_control(2);
				point.x = 380;		//���ֵ�����
				point.y = li_cang[2];
				point.total_angle = 110;
				claw_control(470);
				arm_current_step ++;
				break;
			case 1:
				extra_time = 0;
			
				point.x = 480;
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				claw_control(540);	
				arm_current_step ++;
				break;
			case 3:	
				extra_time = 0;
			
				point.y = li_cang[2] - 50;
				point.total_angle = 105;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 300;
			
				claw_control(400);	
				arm_current_step ++;
				break;
			case 5:
				extra_time = 0;
			
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				point.x = 410;
				point.total_angle = 130;
				point.y = 80;
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 8:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				huadao_control(1);
				break;
		}}

		
void li_cang_put(uint8_t li_cang_num)//arm_control_mode = 6 5
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;			
				claw_control(510);
				tulun_control(1);
				huadao_control(0);//���ַ��򻬵��Ƕ�
				arm_current_step ++;
				break;
			case 1:
				extra_time = 300;
			
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				extra_time = 300;
			
				claw_control(400);	//claw��ȡ
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 3://����ӣ�������
				extra_time = 0;
			
				point.x = 275;
			    point.y = 180;
				point.total_angle = 180;
				tulun_control(0);
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				point.x = 300;
			    point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:	
				extra_time = 0;
			
				point.x = 380;		//���ֵ�����
				point.y = li_cang[li_cang_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				point.x = 470;
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				claw_control(470);
				point.y = li_cang[li_cang_num] - 40;
				arm_current_step ++;
				break;
			case 8:
				extra_time = 0;
			
				point.x = 380;
				arm_current_step ++;
				break;
			case 9:
				extra_time = 0;
			
				set_normal_pos();//�ع�
				claw_control(600);
				huadao_control(1);
				arm_current_step ++;
				break;
			case 10:			
				huadao_control(1);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void li_cang_put_diceng(void)//arm_control_mode = 4
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				claw_control(510);
				tulun_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				extra_time = 300;
			
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				extra_time = 300;
			
				claw_control(400);	//claw��ȡ
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 3:
				extra_time = 0;
			
				point.x = 275;
			    point.y = 180;
				point.total_angle = 180;
				tulun_control(0);
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				point.x = 300;
			    point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:
				extra_time = 0;
				
				huadao_control(2);
				point.x = 380;		//���ֵ�����
				point.y = li_cang[2] - 10;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				point.x = 470;
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				claw_control(470);
				point.y = li_cang[2] - 30;
				arm_current_step ++;
				break;
			case 8:
				extra_time = 0;
			
				point.x = 380;
				point.total_angle = 110;
				point.y = li_cang[2] - 20;
				arm_current_step ++;
				break;
			case 9:
				extra_time = 0;
			
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 10:
				huadao_control(1);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}


void daoduo_put(uint8_t li_cang_num)//arm_control_mode 13 14
{
	switch(arm_current_step)
		{
			case 0:	
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				point.x = 380;		//���ֵ�����
				point.y = li_cang[li_cang_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 1:
				extra_time = 0;
			
				point.x = 480;
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				claw_control(470);
				point.y = li_cang[li_cang_num] - 40;
				arm_current_step ++;
				break;
			case 3:
				extra_time = 0;
			
				point.x = 380;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				set_normal_pos();//�ع�
				claw_control(600);
				arm_current_step ++;
				break;
			case 5:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void daoduo_put_diceng(void)//arm_control_mode = 12
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 300;
			
				arm_ctrl_signal = 1;
				huadao_control(2);
				point.x = 380;		//���ֵ�����
				point.y = li_cang[2];
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 1:
				extra_time = 0;
			
				point.x = 470;
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				claw_control(470);
				point.y = li_cang[2] - 40;
				arm_current_step ++;
				break;
			case 3:
				extra_time = 0;
			
				point.x = 380;
				point.y = li_cang[2] - 20;
				point.total_angle = 110;
				point.y = 80;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 5:
				huadao_control(1);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

float pianzhi_test = 3.3;
float pianzhi_x = 60;
float pianzhi_y = 40;
uint8_t lizhuang_ball_num = 0;
bool_t lizhuang_success_flag = 0;
void lizhuang_shijue_take(void)//�����Ӿ����Ƶ�������ƽ�棬��ͨ�������� //arm_control_mode 10
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				claw_control(490);
				huadao_control(1);
				arm_current_step ++;
				break;
			case 1:
				extra_time = 800;
			
				uint8_t num = 0;
				float distance_total = 0;
				while(num < 100)
				{
					distance_total += shijue_data.ball_y;
					num ++;
					osDelay(1);
				}
				if(fabs(shijue_data.ball_x - 666) > 2)
				{
					point.x += shijue_data.ball_distance * sinf(70*D2R) - shijue_data.ball_y * cosf(70*D2R)-pianzhi_x;
//					point.x += (140.0f - distance_total / 100.0f * pianzhi_test) / tanf(lizhuang_angle) - 95.0f * cosf(lizhuang_angle);
//					point.x += 10.0f + (distance_total/100.0f - piancha) * cosf(lizhuang_angle) - shijue_data.ball_y * sinf(lizhuang_angle);
				}
				else
				{
					arm_shijue_error ++;
				}
				//point.x = 590;
				//point.y -=  shijue_data.ball_distance*cosf(70*D2R) - shijue_data.ball_y * sinf(70*D2R)-pianzhi_y;	
				point.y = 210;//������һ��㣬ԭ��220,ԭ����ԭ��210
				arm_current_step ++;
				break;
			case 2:
				extra_time = 0;
			
				claw_control(400);	//claw��ȡ
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 3:
				extra_time = 300;
			
				point.x = 450;
				point.y = 350;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 300;
			
				point.x = 300;
				point.y = 300;
				if(shijue_data.ball_distance < 180)
				{
					lizhuang_success_flag = 1;
					lizhuang_ball_num ++;
				}
				arm_current_step ++;
				break;
//			case 5:
//				set_bodanpan_pos();
//				arm_current_step ++;
//				break;
			case 5:					//��׼����
				extra_time = 0;
			
				point.x = 360;
				point.y = 200;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				claw_control(470);
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				set_normal_pos();
//				if(lizhuang_success_flag == 1)
//				{
//					lizhuang_success_flag = 0;
//				}
				claw_control(600);
				arm_current_step ++;
				break;
			case 8:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
			
				a_new_ball_in = 1;
				break;
		}
}


extern bool_t zhuanpanji_finish_flag;//ת�̻�������־λ
void zhuanpanji_take(void)//arm_control_mode 11
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				claw_control(510);
				tulun_control(0);
				huadao_control(3);
				bogan_control(2);
				arm_current_step ++;
				break;
			
			case 1:
				extra_time = 300;
			
				point.x = 300;	
				point.y = 350;
				point.total_angle = 85;
				bogan_control(2);
				arm_current_step ++;
				break;
			
			case 2:
				extra_time = 300;
			
				point.x = 420;	//Բ�̻������x������420
				point.y = 350;
				point.total_angle = 85;
				bogan_control(2);
				arm_current_step ++;
				break;
			case 3:
				extra_time = 0;
			
				point.x = 410;	//Բ�̻�x����,435
				point.y = 250;
				point.total_angle = 85;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 300;
			
				if(zhuanpanji_finish_flag == 1)
				{
					point.x = 445;
					point.y = 320;
					point.total_angle = 85;
					bogan_control(2);
					arm_current_step ++;
					break;
				}
				break;
			case 5:
				extra_time = 0;
			
				bogan_control(0);
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 6:
				extra_time = 1000;
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void gan_rao_qiu(void)//������ȥ����ƽ̨ arm_control_mode 15
{
	switch(arm_current_step)
		{
			case 0:
				extra_time = 0;
			
				arm_ctrl_signal = 1;
				claw_control(490);
				tulun_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				extra_time = 300;
			
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				extra_time = 300;
			
				claw_control(400);	//claw��ȡ
				arm_current_step ++;
				//����Լ��Ƿ�е�����ж�
				break;
			case 3://����ӣ�������
				extra_time = 300;
			
				point.x = 275;
			    point.y = 180;
				point.total_angle = 180;
				arm_current_step ++;
				break;
			case 4:
				extra_time = 0;
			
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:
				extra_time = 0;
				tulun_control(0);
				point.x = 500;		//���ֵ�����
				point.y = jie_ti_ping_tai[0] + 40;
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 6:
				extra_time = 0;
			
				claw_control(470);	
				arm_current_step ++;
				break;
			case 7:
				extra_time = 0;
			
				set_normal_pos();
				claw_control(600);
				arm_current_step ++;
				break;
			case 8:
				huadao_control(1);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
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
			//�ý���ƽ̨����1��ߣ�2�м䣬3���
			case 1:
				jie_ti_ping_tai_take_dingceng();
				break;
			case 2:
				jie_ti_ping_tai_take(1);
				break;
			case 3:
				jie_ti_ping_tai_take(2);
				break;
			//����������֣�4��ͣ�5�м䣬6���
			case 4:
				li_cang_put_diceng();	//licang_current_line = 1����һ��
				break;
			case 5:
				li_cang_put(1);
				break;
			case 6:
				li_cang_put(0);
				break;
			
			//������������ó�(����)
			case 7:
				li_cang_take(0);
				break;
			case 8:
				li_cang_take(1);
				break;
			case 9:
				li_cang_take_diceng();
				break;
			
			case 10:
				lizhuang_shijue_take();
				break;
			case 11:
				zhuanpanji_take();
				break;
			
			//��������������֣�12��ͣ�13�м䣬14���
			case 12:
				daoduo_put_diceng();	//licang_current_line = 1����һ��
				break;
			case 13:
				daoduo_put(1);
				break;
			case 14:
				daoduo_put(0);
				break;
			case 15:
				gan_rao_qiu();
				break;
			//������
			case 66:
				arm_ctrl_signal = 1;
				break;
					
		}
		//ȷ����е�ۺͶ��������񣬿ɶ�̬����
		osDelay((uint32_t)servo_Data.serial_servo_Time + (uint32_t)extra_time);
	}
}
