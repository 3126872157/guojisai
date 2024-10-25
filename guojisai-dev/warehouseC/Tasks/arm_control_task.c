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
float piancha = 75;//摄像头到夹爪的距离偏差


//从高往低0,1,2
float jie_ti_ping_tai[3] = {260, 220, 160};
//从高往低0,1,2
float li_cang[3] = {330, 210, 90};//{355, 235, 115}
float jieti_x = 490;//阶梯平台x
float lizhuang_x =  250.0f;
float lizhuang_angle = 20.0f * D2R;

uint8_t arm_control_mode = 0;
uint8_t arm_current_step = 0;

uint16_t extra_time = 1000;

void set_bodanpan_pos(void)
{
	point.x = 295;
	point.y = -45;
	point.total_angle = 175;
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

//extra_time 0，arm_slow_start_k 0.001

void jie_ti_ping_tai_take_dingceng(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 1000;
				claw_control(490);
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
				
				point.x = jieti_x;		//阶梯的坐标
				point.y = jie_ti_ping_tai[0];
				point.total_angle = 95;
				arm_current_step ++;
				break;
			case 2:
				claw_control(410);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:					//缓冲
				set_jieti_middle_pos();
				arm_current_step ++;
				break;
			case 4:					//缓冲
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:					//对准拨蛋盘
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 6:					//放球
				claw_control(470);	
				arm_current_step ++;
				break;
			case 7:					//回归
				set_normal_pos();
				arm_current_step ++;
				a_new_ball_in = 1;
				break;
			case 8:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				extra_time = 1000;
				break;
		}
}


void jie_ti_ping_tai_take(uint8_t jie_ti_num)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 800;
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
			
				point.x = jieti_x;		//阶梯的坐标
				point.y = jie_ti_ping_tai[0] - 20;
				claw_control(400);	//担心碰到边沿，先关闭夹爪
				point.total_angle = 95;
				arm_current_step ++;
				break;
				
			case 1:
				point.y = jie_ti_ping_tai[jie_ti_num];
				claw_control(520);	//张开夹爪
				arm_current_step ++;
				break;
				
			case 2:
				claw_control(410);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:					//缓冲
				set_jieti_middle_pos();
				arm_current_step ++;
				break;
			case 4:					//缓冲
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:					//对准拨蛋盘
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 6:					//放球
				claw_control(470);	
				arm_current_step ++;
				break;
			case 7:					//回归
				set_normal_pos();
				arm_current_step ++;
				a_new_ball_in = 1;
				break;
			case 8:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				extra_time = 1000;
				break;
		}
}

void li_cang_take(uint8_t li_cang_num)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 300;
				point.x = 380;		//立仓的坐标
				point.y = li_cang[li_cang_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 1:
				claw_control(470);	
				arm_current_step ++;
				break;
			case 2:
				point.x = 480;
				arm_current_step ++;
				break;
			case 3:		
				claw_control(510);	
				point.y = li_cang[li_cang_num] - 50;
				arm_current_step ++;
				break;
			case 4:
				claw_control(410);	
				arm_current_step ++;
				break;
			case 5:
				point.y = li_cang[li_cang_num];
				arm_current_step ++;
				break;
			case 6:
				point.x = 380;
				arm_current_step ++;
				break;
			case 7:
				claw_control(410);
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
				break;
		}
}

void li_cang_take_diceng(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				huadao_control(2);
				extra_time = 500;
				point.x = 380;		//立仓的坐标
				point.y = li_cang[2];
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 1:
				point.x = 480;
				arm_current_step ++;
				break;
			case 2:
				claw_control(510);	
				arm_current_step ++;
				break;
			case 3:	
				point.y = li_cang[2] - 40;
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 4:
				claw_control(410);	
				arm_current_step ++;
				break;
			case 5:
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 6:
				point.x = 410;
				point.total_angle = 130;
				point.y = 80;
				arm_current_step ++;
				break;
			case 7:
				claw_control(410);
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
				huadao_control(0);
				break;
		}}

void li_cang_put(uint8_t li_cang_num)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 300;
				claw_control(600);
				tulun_control(1);
				arm_current_step ++;
				break;
			case 1:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				claw_control(410);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:
				set_normal_pos();
				tulun_control(0);
				arm_current_step ++;
				break;
			case 4:	
				point.x = 380;		//立仓的坐标
				point.y = li_cang[li_cang_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 5:
				point.x = 480;
				arm_current_step ++;
				break;
			case 6:
				claw_control(470);
				arm_current_step ++;
				break;
			case 7:
				point.x = 380;
				arm_current_step ++;
				break;
			case 8:
				set_normal_pos();
				arm_current_step ++;
				break;
			case 9:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void daoduo_put(uint8_t li_cang_num)
{
	switch(arm_current_step)
		{
			case 0:	
				arm_ctrl_signal = 1;
				extra_time = 300;
				point.x = 380;		//立仓的坐标
				point.y = li_cang[li_cang_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 1:
				point.x = 480;
				arm_current_step ++;
				break;
			case 2:
				claw_control(470);
				arm_current_step ++;
				break;
			case 3:
				point.x = 380;
				arm_current_step ++;
				break;
			case 4:
				set_normal_pos();
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

void li_cang_put_diceng(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 500;
				claw_control(600);
				tulun_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				claw_control(410);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:
				set_normal_pos();
				tulun_control(0);
				arm_current_step ++;
				break;
			case 4:
				huadao_control(2);
				point.x = 380;		//立仓的坐标
				point.y = li_cang[2];
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 5:
				point.x = 480;
				arm_current_step ++;
				break;
			case 6:
				claw_control(470);	
				arm_current_step ++;
				break;
			case 7:
				point.x = 410;
				point.total_angle = 130;
				point.y = 80;
				arm_current_step ++;
				break;
			case 8:
				set_normal_pos();
				arm_current_step ++;
				break;
			case 9:
				huadao_control(0);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void daoduo_put_diceng(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 500;
				huadao_control(2);
				point.x = 380;		//立仓的坐标
				point.y = li_cang[2];
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 1:
				point.x = 480;
				arm_current_step ++;
				break;
			case 2:
				claw_control(470);	
				arm_current_step ++;
				break;
			case 3:
				point.x = 410;
				point.total_angle = 130;
				point.y = 80;
				arm_current_step ++;
				break;
			case 4:
				set_normal_pos();
				arm_current_step ++;
				break;
			case 5:
				huadao_control(0);
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

float pianzhi_test = 3.3;
uint8_t lizhuang_ball_num = 0;
bool_t lizhuang_success_flag = 0;
void lizhuang_shijue_take(void)//先用视觉横移到球所在平面，再通过测距夹球
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 800;
				claw_control(490);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
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
					point.x += (140.0f - distance_total / 100.0f * pianzhi_test) / tanf(lizhuang_angle) - 95.0f * cosf(lizhuang_angle);
//					point.x += 10.0f + (distance_total/100.0f - piancha) * cosf(lizhuang_angle) - shijue_data.ball_y * sinf(lizhuang_angle);
				}
				else
				{
					arm_shijue_error ++;
				}
				//point.x = 590;b
				point.y = 210;	//调高了一点点，原本220,原本的原本210
				arm_current_step ++;
				break;
			case 2:
				claw_control(400);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:
				point.x = 450;
				point.y = 350;
				arm_current_step ++;
				break;
			case 4:
				point.x = 300;
				point.y = 300;
				if(shijue_data.ball_distance < 180)
					lizhuang_success_flag = 1;
					lizhuang_ball_num ++;
				arm_current_step ++;
				break;
			case 5:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 6:
				claw_control(470);
				arm_current_step ++;
				break;
			case 7:
				set_normal_pos();
//				if(lizhuang_success_flag == 1)
//				{
					a_new_ball_in = 1;
//					lizhuang_success_flag = 0;
//				}
				arm_current_step ++;
				break;
			case 8:
				arm_control_mode = 0;
				extra_time = 1000;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}


extern bool_t zhuanpanji_finish_flag;//转盘机结束标志位
void zhuanpanji_take(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 1000;
				claw_control(510);
				tulun_control(0);
				huadao_control(1);
				bogan_control(2);
				arm_current_step ++;
				break;
			case 1:
				point.x = 420;
				point.y = 280;
				point.total_angle = 85;
				bogan_control(2);
				arm_current_step ++;
				break;
			case 2:
				point.x = 435;
				point.y = 230;
				point.total_angle = 85;
				arm_current_step ++;
				break;
			case 3:
				if(zhuanpanji_finish_flag == 1)
				{
					point.x = 445;
					point.y = 280;
					point.total_angle = 85;
					bogan_control(2);
					arm_current_step ++;
					break;
				}
				break;
			case 4:
				bogan_control(0);
				set_normal_pos();
				arm_current_step ++;
				break;
			case 5:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
				break;
		}
}

void gan_rao_qiu(void)//干扰球去阶梯平台
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 500;
				claw_control(490);
				tulun_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				claw_control(410);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 3:
				set_normal_pos();
				tulun_control(0);
				arm_current_step ++;
				break;
			case 4:
				point.x = 500;		//立仓的坐标
				point.y = jie_ti_ping_tai[2] + 20;
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 5:
				claw_control(470);	
				arm_current_step ++;
				break;
			case 6:
				set_normal_pos();
				arm_current_step ++;
				break;
			case 7:
				huadao_control(0);
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
			//普通模式
			case 0:
				set_normal_pos();
				break;
			//拿阶梯平台的球：1最高，2中间，3最低
			case 1:
				jie_ti_ping_tai_take_dingceng();
				break;
			case 2:
				jie_ti_ping_tai_take(1);
				break;
			case 3:
				jie_ti_ping_tai_take(2);
				break;
			//将球放入立仓：4最低，5中间，6最高
			case 4:
				li_cang_put_diceng();	//licang_current_line = 1，第一行
				break;
			case 5:
				li_cang_put(1);
				break;
			case 6:
				li_cang_put(0);
				break;
			
			//将球从立仓中拿出(倒垛)
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
			
			//将倒垛球放入立仓：12最低，13中间，14最高
			case 12:
				daoduo_put_diceng();	//licang_current_line = 1，第一行
				break;
			case 13:
				daoduo_put(1);
				break;
			case 14:
				daoduo_put(0);
				break;
			
			//调试用
			case 66:
				arm_ctrl_signal = 1;
				break;
					
		}
		//确保机械臂和舵机完成任务，可动态调整
		osDelay((uint32_t)servo_Data.serial_servo_Time + (uint32_t)extra_time);
	}
}
