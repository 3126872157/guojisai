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

//视觉有关参数
extern shijue_Data shijue_data;
int8_t shi_jue_x_pianzhi = 5;
float shijue_k = -1;
uint8_t shijue_error = 0;
float shijue_suoqiu_tolerance = 5;//视觉锁球忍耐值
float shijue_suozhang_tolerance = 5;//视觉锁障忍耐值(用于避障前的精准中心定位)
float obstacle_x_tol = 280;//用于大幅度横移过程中的锁定障碍
float obstacle_distance_tol = 300;//在障碍物前多少距离停下
extern uint8_t TX_shijue_mode;


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
8：无力状态，用于debug控制
********************************************************/

//机械臂任务开始的标志位
bool_t mode4_task_start = 0;
bool_t mode5_task_start = 0;
bool_t mode9_task_start = 0;
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
	{1,		 180,	 	10,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//走到立桩前
	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //横移视觉锁球
	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球
	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //横移视觉锁球
	{5,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//测距模式夹球
	
	
	{11,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
	//底盘立桩到阶梯平台过渡
	{1,		 -20,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//立桩完成，后退20
	{1,		 0,		  	0,			-90,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//顺时针转90
	{1,		 90,	 	10,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//大致来到阶梯平台右侧
	{2,		 5,	      	0,			0,			non,			CHASSIS_V},				 //前进灰度识别白线后停
	
	
	{11,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
	//阶梯平台
	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
	{4,		 non,	    non,	    non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第一个
	
	{1,		 0,	 		10,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//往左走10，以防视觉看不到球
	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
	{4,		 non,	 	non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第二个
	
	{1,		 0,	 		10,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//往左走10，以防视觉看不到球
	{3,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁球
	{4,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//夹取第三个
	
	
	{11,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
	//阶梯平台到圆盘机过渡
	{1,		 -20,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//阶梯平台完成，后退20
	{1,		 0,	 		0,			180,		gyro_tol,		CHASSIS_MOVE_AND_ROTATE},//逆时针旋转180，以便视觉锁障
	{6,		 0,		  	-160,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//锁障，若跑到160，说明没锁上，停下
	{7,		 100,		 0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//锁障后向前，待距离小于给定值，作避障动作
	{8,		 0,		  	5,			0,			non,			CHASSIS_V},				 //视觉横移锁障
	{1,		 0,	 		40,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作
	{1,		 70,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作
	{1,		 0,	 		-40,		0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//避障动作
	{2,		 10,	     0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停
	
	
	//圆盘机
	{9,		 non,		non,		non,		non,			CHASSIS_MOVE_AND_ROTATE},//机械臂就位,拨杆拨球
	
	{8,		 non,	 	non,		non,		non,			CHASSIS_V},//用来调试，不需要就删掉
	//圆盘机到仓库过渡
	{1,		 -10,	 	0,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//后退10
	{1,		 0,	 	  220,			0,			distance_tol,	CHASSIS_MOVE_AND_ROTATE},//应该要配合二维码锁定位置(待增加)
	{2,		 5,	      	0,			0,			non,			CHASSIS_V},//前进灰度识别白线后停
	
	
	//仓库
	
	
	
	
	{8,		 non,	 	non,		non,		non,			CHASSIS_V},
	
	
};

//当前所在位置序号
uint8_t currentTargIndex = 0;
//完成任务标志
uint8_t isFinished = 0;


bool_t flag;

uint16_t test_pos = 900;
uint16_t test_pos_2 = 600;
void flow_task(void const * argument)
{
	while(1)
	{
		if(shijue_data.ball_distance == 1) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, test_pos);//250
		else __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 250);
		__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, test_pos_2);//600放球(范围250-1250)
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
				chassis_move.gyro_set += target.para3;
				//判断误差
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
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
				
				//逻辑待完善
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
				//球在左边
				if(shijue_data.ball_x < 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = target.para2;
				}
				//球在右边
				else if(shijue_data.ball_x > 0 && fabs(shijue_data.ball_x - 666) > 2)
				{
					V_mode_y_speed = -target.para2;
				}
				
				//发666过来表示识别不到
				else if(fabs(shijue_data.ball_x - 666) < 2)
				{	
					V_mode_y_speed = 0;
					shijue_error ++;
				}
				
				if(fabs(shijue_data.ball_x + shi_jue_x_pianzhi) < shijue_suoqiu_tolerance)
				{
					chassis_code_reset_flag = 1;
					V_mode_y_speed = 0;
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
						mode4_task_start = 1;
					}
					else if(shijue_data.ball_y > 9/*距离为最低一层*/)
					{
						arm_control_mode = 3;
						mode4_task_start = 1;
					}
				}
				if(arm_control_mode == 0)
				{
					a_new_ball_in = 1;
					mode4_task_start = 0;
					currentTargIndex ++;
				}
			}	
			
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
			
			else if(target.mode == 6)
			{
				//设置底盘运动目标
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				chassis_move.vy_max_speed = 20.0f;//锁球会有滞后性，所以这里速度限幅一下
				chassis_move.vy_min_speed = -20.0f;//锁球若不在正中心可以考虑用底盘横移微调一下(视具体情况再决定是否添加)
				
				if(fabs(shijue_data.obstacle_x) < obstacle_x_tol)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					
					chassis_move.vy_max_speed = 50.0f;
					chassis_move.vy_min_speed = -50.0f;
					currentTargIndex ++;
				}
				
				float distance = sqrt(pow(target.para1 - chassis_move.x, 2) + pow(target.para2 - chassis_move.y, 2));
				if(distance < distance_tol && fabs(chassis_move.gyro - target.para3) < gyro_tol)
				{
					//里程计清零
					chassis_code_reset_flag = 1;
					currentTargIndex ++;
				}
			}
			
			else if(target.mode == 7)
			{
				//设置底盘运动目标
				TX_shijue_mode = 2;
				chassis_behaviour_mode = target.chassis_mode;
				chassis_move.x_set = target.para1;
				chassis_move.y_set = target.para2;
				chassis_move.gyro_set = target.para3;
				chassis_move.vx_max_speed = 20.0f;////锁球会有滞后性，所以这里速度限幅一下
				chassis_move.vx_min_speed = -20.0f;
				if(fabs(shijue_data.obstacle_distance) < obstacle_distance_tol)
				{
					//里程计清零			
					chassis_code_reset_flag = 1;
					chassis_move.vx_max_speed = 50.0f;
					chassis_move.vx_min_speed = -50.0f;
					currentTargIndex ++;
				}
				
			}
			
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
			
			else if(target.mode == 11)
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
