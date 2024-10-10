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
float jie_ti_ping_tai[3] = {245, 190, 145};
//从高往低0,1,2
float li_cang[3] = {345, 225, 105};
float lizhuang_x =  250.0f;
float lizhuang_angle = 20.0f * D2R;

uint8_t arm_control_mode = 0;
uint8_t arm_current_step = 0;

uint16_t extra_time = 1000;

void set_bodanpan_pos(void)
{
	point.x = 295;
	point.y = -35;
	point.total_angle = 175;
}

void set_normal_pos(void)
{
	claw_control(1);
	point.x = 300;
	point.y = 300;
	point.total_angle = 110;
	tulun_control(0);
}

void set_jieti_middle_pos(void)
{
	point.x = 400;
	point.y = 400;
	point.total_angle = 75;
}

//extra_time 0，arm_slow_start_k 0.001
void jie_ti_ping_tai_take(uint8_t jie_ti_num)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				claw_control(2);	//claw中间位置
				point.x = 495;		//阶梯的坐标
				point.y = jie_ti_ping_tai[jie_ti_num];
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 1:
				claw_control(0);	//claw夹取
				arm_current_step ++;
				//这可以加是否夹到球的判断
				break;
			case 2:					//缓冲
				set_jieti_middle_pos();
				arm_current_step ++;
				break;
			case 3:					//缓冲
				point.x = 300;
				point.y = 300;
				point.total_angle = 110;
				arm_current_step ++;
				break;
			case 4:					//对准拨蛋盘
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 5:					//放球
				claw_control(1);	
				arm_current_step ++;
				break;
			case 6:					//回归
				set_normal_pos();
				arm_current_step ++;
				a_new_ball_in = 1;
				break;
			case 7:
				arm_control_mode = 0;
				arm_current_step = 0;
				arm_ctrl_signal = 0;
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
				claw_control(1);	
				arm_current_step ++;
				break;
			case 2:
				point.x = 480;
				arm_current_step ++;
				break;
			case 3:	
				point.y = li_cang[li_cang_num] - 50;
				arm_current_step ++;
				break;
			case 4:
				claw_control(0);	
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
				set_normal_pos();
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
				claw_control(1);	
				arm_current_step ++;
				break;
			case 3:	
				point.y = li_cang[2] - 40;
				point.total_angle = 90;
				arm_current_step ++;
				break;
			case 4:
				claw_control(0);	
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
				set_normal_pos();
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
				claw_control(1);
				tulun_control(1);
				arm_current_step ++;
				break;
			case 1:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				claw_control(0);	//claw夹取
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
				claw_control(2);
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

void li_cang_put_diceng(void)
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 500;
				claw_control(1);
				tulun_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 2:
				claw_control(0);	//claw夹取
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
				claw_control(2);	
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


void lizhuang_shijue_take(void)//先用视觉横移到球所在平面，再通过测距夹球
{
	switch(arm_current_step)
		{
			case 0:
				arm_ctrl_signal = 1;
				extra_time = 1000;
				claw_control(1);
				huadao_control(0);
				arm_current_step ++;
				break;
			case 1:
				//如果识别到球
				if(fabs(shijue_data.ball_x - 666) > 2)
				{
					point.x += (shijue_data.ball_distance - piancha) * cosf(lizhuang_angle) - shijue_data.ball_y * sinf(lizhuang_angle);
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
				claw_control(0);	//claw夹取
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
				arm_current_step ++;
				break;
			case 5:
				set_bodanpan_pos();
				arm_current_step ++;
				break;
			case 6:
				claw_control(2);
				arm_current_step ++;
				break;
			case 7:
				set_normal_pos();
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
				jie_ti_ping_tai_take(0);
				break;
			case 2:
				jie_ti_ping_tai_take(1);
				break;
			case 3:
				jie_ti_ping_tai_take(2);
				break;
			//将球放入立仓：4最高，5中间，6最低
			case 4:
				li_cang_put(0);
				break;
			case 5:
				li_cang_put(1);
				break;
			case 6:
				li_cang_put_diceng();
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
				lizhuang_shijue_take();//先用视觉横移到球所在平面，再通过测距走到可夹球位置
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
