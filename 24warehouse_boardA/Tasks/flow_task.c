#include "flow_task.h"
#include "chassis_behaviour.h"
#include "bodanpan.h"
#include "Bodanpan_Task.h"
#include "arm_control_task.h"
#include "arm_ctrl.h"

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

//灰度传感器数据，0为白色红色，1为黑色，目前受硬件限制只有一路数据gray_data[0]
extern uint8_t gray_data[2];

//机械臂控制有关参数
extern uint8_t arm_control_mode;
extern bool_t arm_ctrl_signal;
extern uint8_t arm_current_step;
extern uint16_t claw_middle_pos;

//视觉有关参数
extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;
float shijue_k = -1;
uint8_t shijue_error = 0;
float shijue_suoqiu_tolerance = 5;//视觉锁球忍耐值
float shijue_suozhang_tolerance = 5;//视觉锁障忍耐值(用于避障前的精准中心定位)
float obstacle_x_tol = 10;//用于大幅度横移过程中的锁定障碍
float QR_x_tol = 10;//锁定二维码忍耐值
float obstacle_distance_tol = 250;//在障碍物前多少距离停下
extern uint8_t TX_shijue_mode;
uint8_t licang_current_line;//仓库层数记录
uint8_t QR_code[4];//0、1、2分别记录从左到右QR值，第四个用不到
uint8_t QR_num;
uint8_t QR_doing[3] = {0};
uint8_t QR_doing_num = 0;

bool_t take_a_ball = 0;
uint8_t ball_x;
uint8_t ball_y;

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
bool_t mode4_task_start = 0;
bool_t mode5_task_start = 0;
bool_t mode9_task_start = 0;
bool_t mode11_task_start = 0;
bool_t bogan_zhunbei_flag = 1;
bool_t bogan_jiqiu_flag = 0;
uint8_t zhuanpanji_ball_num = 0;

//控制变量列表，格式如下
//控制模式		para1		para2		para3		到达判断误差值			底盘运动模式
//0				
//1				x			y       	gyro   		distance_tol或gyro_tol	CHASSIS_MOVE_AND_ROTATE
//2				x_speed		y_speed											CHASSIS_V
//3				x_speed		y_speed											CHASSIS_V
//4																			CHASSIS_MOVE_AND_ROTATE
//5

const uint8_t non = 0;

TargetPoints targ_point[] = {
		//立桩
/*0*/	{1,		 0,	 		35,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//走到立桩前
/*1*/	{1,		 195,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//走到立桩前
/*2*/	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},				 //横移视觉锁球
/*3*/	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球
/*4*/	{3,		 0,		  	-5,			0,			non,			CHASSIS_V},				 //横移视觉锁球
/*5*/	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球

		//底盘立桩到阶梯平台过渡
/*6*/	{1,		 -20,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//立桩完成，后退20
/*7*/	{1,		 0,		  	0,			-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//顺时针转90
/*8*/	{1,		 60,	 	0,			-90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//大致来到阶梯平台右侧
/*9*/	{1,		 0,	 		8,			-90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//大致来到阶梯平台右侧
/*10*/	{2,		 5,	      	0,			0,			non,			CHASSIS_V},				 //前进灰度识别白线后停

		//阶梯平台
/*11*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
/*12*/	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第一个
	
/*13*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
/*14*/	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第二个
	
/*15*/	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
/*16*/	{4,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第三个
	
	
//	{66,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
		//阶梯平台到圆盘机过渡
/*17*/	{1,		 -20,	 	0,			-90,		distance_tol,	CHASSIS_MOVE_AND_ROTATE},//阶梯平台完成，后退20
/*18*/	{1,		 0,	 		0,			90,			gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//逆时针旋转180，以便视觉锁障
/*19*/	{1,		 0,		  	-80,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//锁障前
/*20*/	{6,		 0,		  	-5,			0,			non,			CHASSIS_V},//缓慢平移锁障
/*21*/	{7,		 5,		 	0,			0,			non,			CHASSIS_V},//锁障后向前，待距离小于给定值，作避障动作
/*22*/	{8,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁障
/*23*/	{1,		 0,	 		40,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作
/*24*/	{1,		 90,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作
/*25*/	{1,		 0,	 		-40,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作，这里会偏一点，加一点补偿，误识别十字当做白线要处理一下
/*26*/	{2,		 10,	     0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停
	
		//圆盘机
/*27*/	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//机械臂就位,拨杆拨球
	
/*28*/	{8,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
		//圆盘机到仓库过渡
/*29*/	{1,		 -15,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退15
/*30*/	{1,		 0,	 		140,		90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//左移130cm，差不多到立仓

		//立仓倒垛（重复三次）
/*31*/	{10,	 0,	      	5,			0,			non,			CHASSIS_V},//视觉平移记录3个二维码对应的位置 ，向左移动
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//视觉横移锁球
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//底盘前进灰度识别线
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
		{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，最好慢一点，并根据二维码数字记录当前列数
/*36*/	{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//锁最右边的二维码，定位用
/*37*/	{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停，然后放球
		//放球
/*38*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，来锁球和二维码，防止偷窥注意距离
		{12,	0,	 		5,			90,			distance_tol,	CHASSIS_V},//左移，锁特定二维码
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//视觉横移锁球
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//底盘前进灰度识别线
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
/*43*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，并根据二维码数字记录当前列数
		{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//锁最右边的二维码，定位用
		{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停，然后放球
		//放球
/*47*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，来锁球和二维码
		{12,	0,	 		5,			90,			distance_tol,	CHASSIS_V},//左移，锁特定二维码
		{3,		 0,		  	5,			0,			non,			CHASSIS_V},//视觉横移锁球
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//底盘前进灰度识别线
		{11,	 non,	    non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹球
/*50*/	{1,		 -20,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，以识别二维码，并根据二维码数字记录当前列数
		{14,	 0,	      	-5,			0,			non,			CHASSIS_V},//锁最右边的二维码，定位用
		{13,	 0,	 		-20,		90,			distance_tol,	CHASSIS_V},//右移到倒垛位，可以单开一个倒垛模式，让速度变慢
		{2,		 5,	     	0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停，然后放球
		//放球

/*34*/
/*35*/
/*36*/
/*37*/

		//立仓放球
/*40*/	{3,		 0,	      	5,			0,			non,			CHASSIS_V},//从最右边往左走，逐个锁二维码
/*41*/	{2,		 5,	     	0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停，放三个球
/*42*/	{1,		 -15,	 	0,			90,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退，锁二维码，重复
		
	

		{66,		 non,	 	non,		non,		non,			CHASSIS_V}
	
	
};

//当前所在位置序号
uint8_t currentTargIndex = 0;
//完成任务标志
uint8_t isFinished = 0;


bool_t flag;

uint16_t test_pos = 250;
uint16_t test_pos_2 = 600;
void flow_task(void const * argument)
{
	while(1)
	{
//		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, test_pos);
		//判断所有步骤是否走完
		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
		{
			//读出当前目标
			TargetPoints target = targ_point[currentTargIndex];
			
			//底盘运动
			if(target.mode == 1)
			{
				//设置底盘运动目标
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					
					//立仓用，这里要求视觉不要666
					if(currentTargIndex >= 29 && currentTargIndex <= 60)
					{
						for(int i  = 0;i<3;i++)
						{
							//记录当前倒垛的列数
							if(shijue_data.QR_code == QR_code[i])
							{
								QR_doing[i] = 1;
							}
						}
					}
					
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//检测到白线就停
			else if(target.mode == 2)
			{
				chassis_behaviour_mode = target.chassis_mode;
				gray_sensor_read();
				V_mode_x_speed = target.para1;
				
				//逻辑待完善！！！！！！！！！！！！！！！！！
				if(gray_data[0] == 0)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					currentTargIndex ++;
				}
			}
			//横移视觉找球
			else if(target.mode == 3)
			{
				chassis_behaviour_mode = target.chassis_mode;
				TX_shijue_mode = 0;
//				V_mode_x_speed = target.para1;
//				V_mode_y_speed = target.para2;
				
				//球在左边
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = target.para2;
				}
				//球在右边，反方向动
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = -target.para2;
				}
				
				//发666过来表示识别不到，按照给定方向动
				else if(fabs(shijue_data.ball_x - 666) < 2)
				{	
					V_mode_y_speed = target.para2;
					shijue_error ++;
				}

				if(V_mode_y_speed < 0)
				{	
					//根据左右方向调整偏置
					shi_jue_x_pianzhi = -shi_jue_x_pianzhi;
				}
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					V_mode_x_speed = 0;
					V_mode_y_speed = 0;
					
					//这里可以加结束判断条件，任务就是左右扫球，或者直接让currentTargetIndex指向另一个步骤
					//可以多开一个last currentTargIndex
					currentTargIndex ++;
				}
			}
			//阶梯平台
			else if(target.mode == 4)
			{
				chassis_behaviour_mode = target.chassis_mode;
				TX_shijue_mode = 0;
				if(mode4_task_start == 0)
				{
					if(shijue_data.ball_y < 0)/*距离为最高一层*/
					{
						arm_control_mode = 1;
						mode4_task_start = 1;
					}
					
					else if(shijue_data.ball_y < 8 && shijue_data.ball_y > 0/*距离为中间一层*/)
					{
						arm_control_mode = 2;
						claw_middle_pos = 450;
						mode4_task_start = 1;
					}
					else if(shijue_data.ball_y > 9/*距离为最低一层*/)
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
			
			//机械臂立桩取球
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
			
			//视觉锁障
			else if(target.mode == 6)
			{
				//设置底盘运动目标
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				if(fabs(shijue_data.obstacle_x) < obstacle_x_tol)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//避障：靠近障碍物
			else if(target.mode == 7)
			{
				//设置底盘运动目标
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				if(fabs(shijue_data.obstacle_distance) < obstacle_distance_tol)
				{
					//里程计清零			
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
				
			}
			
			//避障：对齐障碍物
			else if(target.mode == 8)
			{
				TX_shijue_mode = 2;			
				chassis_behaviour_mode = target.chassis_mode;
				//障碍在左边
				if(shijue_data.obstacle_x < 0)
				{
					V_mode_y_speed = target.para2;
				}
				//障碍在右边
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
			
			//转盘机机械臂拨球
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
				
				
				/*******这里a_new_ball_in不知道咋加！！！！*******/
				
				
				if(zhuanpanji_ball_num == 5)
				{
					osDelay(500);//这里延迟一下，防止最后一次击球还没成功，机械臂就抬起来了
					arm_control_mode = 0;
					arm_current_step = 0;
					arm_ctrl_signal = 0;
					currentTargIndex ++;
				}

			}
			
			//立仓视觉存二维码
			else if(target.mode == 10)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				
				//二维码存在且跟上次读到的不一样时，从尾部开始存
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
			
			//倒垛夹球
			else if(target.mode == 11)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;
				if(mode11_task_start == 0)
				{
					if(shijue_data.ball_y < -10)//最高一层
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
					else if(shijue_data.ball_y == 666)//最低一层
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
			
			//视觉锁特定二维码
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
			
			//慢速的移动，看里程停下，往右走
			else if(target.mode == 13)
			{
				chassis_behaviour_mode = target.chassis_mode;
				V_mode_x_speed = 0;
				V_mode_y_speed = -5;
				
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			//锁最后的二维码
			else if(target.mode == 14)
			{
				TX_shijue_mode = 0;
				chassis_behaviour_mode = target.chassis_mode;	
				V_mode_x_speed = target.para1;
				V_mode_y_speed = target.para2;
				
				//锁最后一个二维码，列3
				if(shijue_data.QR_code == QR_code[2] && shijue_data.QR_x < 10)
				{
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}

			//测试用
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
			//删除当前任务
			vTaskDelete(NULL);
		}
		osDelay(1);
	}
}
