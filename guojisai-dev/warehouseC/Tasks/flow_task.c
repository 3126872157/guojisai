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
//控制底盘运动
extern chassis_move_t chassis_move;
extern chassis_mode_e chassis_behaviour_mode;
extern bool_t chassis_code_reset_flag;

//CHASSIS_V模式下三个方向给定速度
extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

//取球放入拨蛋盘后的标志位
extern bool_t a_new_ball_in;

//检查模式参数
extern uint8_t lizhuang_ball_num;//立桩夹到了几个球
uint8_t ball_error = 0;

//灰度传感器数据，0为白色红色，1为黑色，目前受硬件限制只有一路数据gray_data[0]
extern uint8_t gray_data[2];

//机械臂控制有关参数
extern uint8_t arm_control_mode;
extern bool_t arm_ctrl_signal;
extern uint8_t arm_current_step;
extern bool_t nan_error;

//视觉有关参数
extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;//暂时先关掉，使用二次锁球
float shijue_k = -1;
uint8_t shijue_error = 0;
extern uint8_t TX_shijue_mode;
uint8_t licang_current_line;//仓库层数记录

float shijue_suoqiu_tolerance = 20;//视觉一次锁球忍耐值mode3
float shijue_suoqiu_tolerance2 = 5;//视觉二次锁球忍耐值mode21
float shijue_suoqiu_tolerance3 = 2;//视觉三次锁球忍耐值mode13

float shijue_suozhang_tolerance = 10;//视觉锁障忍耐值(用于避障前的精准中心定位)
float obstacle_x_tol = 50;//用于横移过程中的锁定障碍
float QR_x_tol = 50;//锁定二维码忍耐值
float obstacle_distance_tol = 250;//在障碍物前多少距离停下

uint8_t QR_code[4];//0、1、2分别记录从左到右QR值，第四个用不到
uint8_t QR_num;
uint8_t QR_doing[3] = {0};
uint8_t QR_PutBall_num = 2;

//防止误识别十字
float bizhang_distance = 0;
float bizhang_distance_total = 130;

bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

//转盘机计时功能
extern uint16_t Time_s;
uint16_t start_time;//转盘机开始时间

//回家
bool_t x_home_finish = 0;
bool_t y_home_finish = 0;


/*******************流程控制模式总览********************
0：机械臂
1：底盘平移加转向位置环
2：底盘配合灰度传感器往前走，检测到白线停
3：底盘配合视觉横向移动，检测到球停
4：机械臂阶梯平台取球
5：机械臂立桩取球
6：底盘平移位置环配合视觉锁障
7：底盘平移位置环配合视觉锁障后前进
8：视觉横移锁障，居中调整
9：转盘机机械臂就位
10：底盘横移锁二维码停
11：立仓倒垛机械臂夹球
66：无力状态，用于debug控制
********************************************************/

//机械臂任务开始的标志位
bool_t modeN_task_start = 0;
bool_t mode9_task_start = 0;
bool_t bogan_zhunbei_flag = 1;
bool_t bogan_jiqiu_flag = 0;
uint8_t zhuanpanji_ball_num = 0;
uint8_t bogan_delay = 150;
bool_t zhuanpanji_finish_flag = 0;

//控制变量列表，格式如下
//控制模式		para1		para2		para3		到达判断误差值			底盘运动模式
//0				
//1				x			y       	gyro   		distance_tol或gyro_tol	CHASSIS_MOVE_AND_ROTATE
//2				x_speed		y_speed											CHASSIS_V
//3				x_speed		y_speed											CHASSIS_V
//4																			CHASSIS_MOVE_AND_ROTATE
//5

const uint8_t non = 0;


float test_v_max = 5;

TargetPoints targ_point[] = {
	
		//起点到立桩
/*0*/	{1,		 182,	 		35,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//走到立桩前

		//立桩拿球
/*1*/	{3,		 0,		  	-100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//横移视觉锁球
		{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//横移视觉二次锁球
		{13,	 0,		  	-100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//横移视觉三次锁球
		{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球

/*5*/	{3,		 0,		  	-100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//横移视觉锁球
		{21,	 0,		  	-100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//横移视觉二次锁球
		{13,	 0,		  	-100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//横移视觉三次锁球
		{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球
		//底盘立桩到阶梯平台过渡
/*9*/	{1,		 -20,	 	0,			0,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//立桩完成，后退20
		{1,		 0,		  	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//顺时针转90
		{1,		 75,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//大致来到阶梯平台右侧
		{1,		 0,	 		20,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//大致来到阶梯平台右侧
		{2,		 100,	     0,			-90,		5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停

		//阶梯平台
/*14*/	{3,		 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//视觉横移锁球
		{21,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//横移视觉二次锁球
		{13,	 0,		  	100,		0,			1,				CHASSIS_MOVE_AND_ROTATE},//横移视觉三次锁球
		{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第一个
	
/*18*/	{3,		 0,		  	100,		0,			6,				CHASSIS_MOVE_AND_ROTATE},//视觉横移锁球
		{21,	 0,		  	100,		0,			3,				CHASSIS_MOVE_AND_ROTATE},//横移视觉二次锁球
		{13,	 0,		  	100,		0,			1.5,			CHASSIS_MOVE_AND_ROTATE},//横移视觉三次锁球
		{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第二个

		//阶梯平台到圆盘机过渡
/*22*/	{1,		 -20,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//阶梯平台完成，后退20
		{1,		 0,	 		0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//逆时针旋转180，以便视觉锁障
		{1,		 0,		  	-120,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//锁障前
		{6,		 0,		  	-200,		0,			10,				CHASSIS_MOVE_AND_ROTATE},//缓慢平移锁障
		{7,		 200,		 0,			0,			5,				CHASSIS_MOVE_AND_ROTATE},//锁障后向前，待距离小于给定值，作避障动作
		{1,		 0,	 		40,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//避障动作
		{1,		 90,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//避障动作
		{1,		 0,	 		-45,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//避障动作，这里会偏一点，加一点补偿，误识别十字当做白线要处理一下
/*30*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
	
		//圆盘机
/*31*/	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//机械臂就位,拨杆拨球
	
		//圆盘机到仓库过渡
/*32*/	{1,		 -5,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退5
/*33*/	{1,		 0,	 		150,		90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//左移150cm，差不多到立仓

		//立仓倒垛（重复三次）
/*34*/	{10,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//视觉平移记录3个二维码对应的位置 ，向左移动
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{19,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//立仓改版视觉横移锁球
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，最好慢一点，并根据二维码数字记录当前列数
/*39*/	{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//锁最右边的二维码，定位用
		{1,	 	 0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		
/*43*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，来锁球和二维码，防止偷窥注意距离
		{16,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//左移，锁列2二维码
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{19,	 0,			100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//立仓改版视觉横移锁球
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，并根据二维码数字记录当前列数
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//锁最右边的二维码，定位用
		{1,	 	0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
/*51*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		
/*53*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，来锁球和二维码
		{14,	 0,	 		100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//左移，锁列3二维码
/*55*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{19,	 0,		  	100,		0,			2,				CHASSIS_MOVE_AND_ROTATE},//立仓改版视觉横移锁球
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
/*58*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，并根据二维码数字记录当前列数
		{14,	 0,	      	-100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//锁最右边的二维码，定位用
		{1,	 	 0,	 		-20,		90,			5,				CHASSIS_MOVE_AND_ROTATE},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{15,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退

/*93*/	{66,		non,	 	non,		non,		non,			CHASSIS_V},

		//立仓放球
/*64*/	{14,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//从最右边往左走，锁列3二维码
/*65*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{18,	 1,/*行号*/	3,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 2,/*行号*/	3,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 3,/*行号*/	3,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
/*69*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，锁二维码
		
/*70*/	{16,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//从最右边往左走，锁列2二维码
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{18,	 1,/*行号*/	2,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 2,/*行号*/	2,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 3,/*行号*/	2,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退，锁二维码
		
/*76*/	{17,	 0,	      	100,		0,			5,				CHASSIS_MOVE_AND_ROTATE},//从最右边往左走，锁列1二维码
		{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
		{18,	 1,/*行号*/	1,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 2,/*行号*/	1,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
		{18,	 3,/*行号*/	1,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球
/*81*/	{1,		 -20,	 	0,			90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退

		//干扰球去阶梯平台
/*82*/	{1,		 0,	 		0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},//旋转180
		{1,		 0,	 		-20,		-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 120,	 	0,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 0,	 		50,			-90,		50.0f,			CHASSIS_MOVE_AND_ROTATE},
/*86*/	{2,		 100,	     0,			90,			5,				CHASSIS_MOVE_AND_ROTATE},//前进灰度识别白线后停
/*87*/	{20,	 1,/*行号*/	4,/*列号*/	non,		non,			CHASSIS_MOVE_AND_ROTATE},//放球

		//回家
		{1,		 -20,	 	0,			-90,			50.0f,			CHASSIS_MOVE_AND_ROTATE},//后退
		{1,		  0,	 	0,			0,			    50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{1,		 -200,	 	55,			0,				50.0f,			CHASSIS_MOVE_AND_ROTATE},
		{22,	   -50,	 	50,			0,				5,				CHASSIS_MOVE_AND_ROTATE},//横向回家
		{23,	   10,	 	-10,		0,				5,				CHASSIS_MOVE_AND_ROTATE},//纵向回家
		{1,		  -3,	 	3,			0,				5,				CHASSIS_MOVE_AND_ROTATE},
/*93*/	{66,	non,	 	non,		non,		non,			CHASSIS_V}
	
	
};

//当前所在位置序号
/******************************************************************************/
uint8_t currentTargIndex = 23;//注意！！！
/******************************************************************************/

//完成任务标志
uint8_t isFinished = 0;


bool_t flag;

uint16_t test_pos = 250;
uint16_t test_pos_2 = 600;
void flow_task(void const * argument)
{
	while(1)
	{
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, test_pos);//拨杆
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, test_pos_2);//滑道
//		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, test_pos);
		//判断所有步骤是否走完
		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
		{
			//读出当前目标
			TargetPoints target = targ_point[currentTargIndex];
			
			//底盘运动
			if(target.mode == 1)
			{
				//设置底盘运动目标
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
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					
					//立仓用，这里要求视觉不要666
//					if(currentTargIndex == 33 || currentTargIndex == 43 || currentTargIndex == 53)
//					{
//						osDelay(100);
//						for(int i  = 0;i < 3;i++)
//						{
//							//记录当前倒垛的列数
//							if(shijue_data.QR_code == QR_code[i])
//							{
//								QR_doing[i] = 1;
//								break;
//							}
//						}
//					}
					
					//里程计清零
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//检测到白线就停
			else if(target.mode == 2)
			{
				//设置底盘运动目标
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
				
				//逻辑待完善！！！！！！！！！！！！！！！！！
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
			//横移视觉找球
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
								
				//球在左边
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//球在右边，反方向动
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//发666过来表示识别不到，按照给定方向动
				else if(fabs(shijue_data.ball_x - 666) < 2)
					chassis_move.y_set = target.para2;

				if(chassis_move.y_set < 0)
					shi_jue_x_pianzhi = -shi_jue_x_pianzhi;//根据左右方向调整偏置
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					//这里可以加结束判断条件，任务就是左右扫球，或者直接让currentTargetIndex指向另一个步骤
					//可以多开一个last currentTargIndex
					currentTargIndex ++;
				}
			}
			//阶梯平台
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					if(shijue_data.ball_y < 5)/*距离为最高一层*/
					{
						arm_control_mode = 1;
						modeN_task_start = 1;
					}
					
					else if(shijue_data.ball_y < 15 && shijue_data.ball_y > 5/*距离为中间一层*/)
					{
						arm_control_mode = 2;
						modeN_task_start = 1;
					}
					else if(shijue_data.ball_y > 15/*距离为最低一层*/)
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
			
			//机械臂立桩取球
			else if(target.mode == 5)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					arm_control_mode = 10;
//					osDelay(100);
//					if(nan_error == 1)//这里要怎么写
					modeN_task_start = 1;
				}
				if(arm_control_mode == 0)
				{
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//视觉锁障
			else if(target.mode == 6)
			{
				//设置底盘运动目标
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
					//里程计清零
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//避障：前进靠近障碍物
			else if(target.mode == 7)
			{
				//设置底盘运动目标
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
					//里程计清零		
					bizhang_distance += chassis_move.x;
					modeN_task_start = 0;
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
				
			}
			
			//避障：对齐障碍物
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
								
				//障碍在左边
				if(shijue_data.obstacle_x < 0)
					chassis_move.y_set = target.para2;
				//障碍在右边，反方向动
				else if(shijue_data.obstacle_x > 0)
					chassis_move.y_set = -target.para2;
								
				if(fabs(shijue_data.obstacle_x) < shijue_suozhang_tolerance)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//转盘机机械臂拨球
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
//					osDelay(500);//这里延迟一下，防止最后一次击球还没成功，机械臂就抬起来了
					zhuanpanji_finish_flag = 1;
				}
				if(arm_control_mode == 0)
				{
					mode9_task_start = 0;
					
					currentTargIndex ++;
				}

			}
			
			//立仓视觉存二维码
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
				
				//二维码存在且跟上次读到的不一样时，从尾部开始存
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
			
			//倒垛夹球
			else if(target.mode == 11)
			{
				chassis_behaviour_mode = target.chassis_mode;
				if(modeN_task_start == 0)
				{
					if(shijue_data.ball_y < -5)//最高一层，原来值为-10
					{
						arm_control_mode = 7;
						modeN_task_start = 1;
						licang_current_line = 3;
					}
					//中间层
					else if(shijue_data.ball_y > 5 && shijue_data.ball_y < 25)//原来值为10和20
					{
						arm_control_mode = 8;
						modeN_task_start = 1;
						licang_current_line = 2;
					}
					else if(fabs(shijue_data.ball_y - 666) < 2)//最低一层
					{
						arm_control_mode = 9;
						modeN_task_start = 1;
						licang_current_line = 1;
					}
					else//最低一层
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
			
			//视觉锁特定二维码
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
			
			//锁列3的二维码
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
				
				//锁最后一个二维码，列3
				if(shijue_data.QR_code == QR_code[2] && fabs(shijue_data.QR_x) < QR_x_tol && fabs(shijue_data.QR_x) > 1)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//机械臂立仓倒垛放球
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
			
			//锁列2二维码，放球
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
			
			//锁列1二维码，放球
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
			
			//立仓放球
			else if(target.mode == 18)
			{
				chassis_behaviour_mode = target.chassis_mode;
				//如果没找到该球说明该球夹取失误，直接跳过本次放球
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
					
				//球在左边
				else if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//球在右边，反方向动
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//发666过来表示识别不到，不动

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance2)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//干扰球放球
			else if(target.mode == 20)
			{
				chassis_behaviour_mode = target.chassis_mode;
				
				//如果没找到该球说明该球夹取失误，直接跳过本次放球
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
			
			//视觉二次横移锁球
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
								
				//球在左边
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//球在右边，反方向动
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//发666过来表示识别不到，按照给定方向动
				else if(fabs(shijue_data.ball_x - 666) < 2)
					chassis_move.y_set = target.para2;

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance2)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//视觉三次横移锁球
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
								
				//球在左边
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = fabs(target.para2);
				//球在右边，反方向动
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
					chassis_move.y_set = -fabs(target.para2);
				
				//发666过来表示识别不到，按照给定方向动
				else if(fabs(shijue_data.ball_x - 666) < 2)
					chassis_move.y_set = target.para2;

				
				if(fabs(shijue_data.ball_x) < shijue_suoqiu_tolerance3)
				{
					chassis_code_reset_flag = 1;
					modeN_task_start = 0;
					currentTargIndex ++;
				}
			}
			
			//回家
			else if(target.mode == 22)
			{
				//设置底盘运动目标
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
				
				if(gray_data[1] == 0 && x_home_finish == 0)//车前灰度
				{
					x_home_finish = 1;
					chassis_move.x_set = 0;
					chassis_move.x = 0;
				}
				if(gray_data[0] == 1 && y_home_finish == 0)//车右灰度
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
				//设置底盘运动目标
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
				
				if(gray_data[1] == 1 && x_home_finish == 0)//车前灰度
				{
					x_home_finish = 1;
					chassis_move.x_set = 0;
					chassis_move.x = 0;
				}
				if(gray_data[0] == 0 && y_home_finish == 0)//车右灰度
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
			
			//测试用
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
				//设置底盘运动目标
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				chassis_move.vx_max_speed = test_v_max;
				chassis_move.vx_min_speed = -test_v_max;
				chassis_move.vy_max_speed = test_v_max;
				chassis_move.vy_min_speed = -test_v_max;
				
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					//里程计清零
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
			//删除当前任务
			vTaskDelete(NULL);
		}
		
		osDelay(1);
	}
}
