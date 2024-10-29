#include "Bodanpan_Task.h"


pid_type_def bodanpan_angle_pid,
             bodanpan_real_angle_pid,
			 bodanpan_speed_pid;

uint8_t ball_num=0;//储蛋机构内已有球的个数
bool_t a_new_ball_in = 0;
extern uint8_t IC_data; //Uart_Task中接收到的，当前位置扭蛋球的IC卡数据
uint8_t err_IC_data = 0xFF;//取球错误标识
extern Pan_t bodanpan;
extern motor_measure_t motor_chassis[5];

//按键有关参数
extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;
extern uint8_t safe_flag;
extern uint8_t arm_safe;
bool_t liucheng_start_flag = 0;

//函数声明
void IC_story(uint8_t data);

//新增的转一圈存球逻辑
bool_t record_start_flag = 0;	//转一圈的时候置1
uint8_t record_num_remain = 0;	//转一圈的时候置10，意为剩下10个球来存
uint8_t record_last = 0;
uint8_t IC_Data_recorded[10] = {0};
uint8_t now_cnt = 0;

//确保IC数据是新的
uint8_t is_IC_data_new(uint8_t record)
{
	if(record == 0)
	{
		return 0;
	}
	int i = 0;
	for(i = 0;i<now_cnt;i++)
	{
		if(record == IC_Data_recorded[i])
		{
			return 0;
		}
	}
	IC_Data_recorded[now_cnt++] = record;
	return 1;
}

void Bodanpan_Task(void const * argument)
{
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	vTaskDelay(CHASSIS_TASK_INIT_TIME);
	bodanpan_motor_init();
	while(1)
	{
		
		/*****************************拨蛋盘task*******************************/
		if(a_new_ball_in)
		{
			bodanpan_position_set(1,1);
//			osDelay(1000);
//			//如果检测到的还是上一个球的数据或者没有检测到,来回动一下，防止IC卡读不到
//			if(IC_data == bodanpan.IC_date_pan[ball_num-1] || IC_data == 0)
//			{
//				bodanpan.angle_set += 1.0f * ANGLE_PER_BOX;
//				bodanpan.code_set = bodanpan.angle_set * 819.2;
//				osDelay(200);
//				bodanpan.angle_set -= 1.0f * ANGLE_PER_BOX;
//				bodanpan.code_set = bodanpan.angle_set * 819.2;
//				osDelay(200);
//				if(IC_data != bodanpan.IC_date_pan[ball_num-1] && IC_data != 0)
//					IC_story(IC_data);
//				else
//					IC_story(err_IC_data);
//			}
//			else IC_story(IC_data);
			//如果还是没有数据，则默认夹球失误，给当前位置存入错误标识err_IC_data
			a_new_ball_in = 0;
//			record_last = IC_data;
		}
		
		/*****************************新增转一圈存球*****************************/
		//数10次
		if(record_num_remain > 0)
		{
			//再次初始化一次拨蛋盘
			if(record_start_flag == 0)
			{
				ball_num = 0;
				bodanpan.position = 0;
				record_start_flag = 1;
			}
			bodanpan_position_set(1,1);	//转一格
			osDelay(1500);
			
			//防止第一个是空的
			if(IC_data != record_last && IC_data != 0 && is_IC_data_new(IC_data))
			{
				
				IC_story(IC_data);
				record_last = IC_data;
			}
			else
			{
				IC_story(err_IC_data);
			}
			
			record_num_remain --;
		}
		
		/*****************************按键和IC_task*****************************/
		
		//按键启动
		if(liucheng_start_flag == 0)
		{
			if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
			{
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
				arm_safe = 0;
				osDelay(2500);
				HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
				safe_flag = 0;
				liucheng_start_flag = 1;
			}
		}
		
		
		osDelay(5);
	}
}



void bodanpan_motor_init(void)
{
	bodanpan_init();
	
	fp32 motor_angle_pan_pid[3] = {BODANPAN_ANGLE_KP, BODANPAN_ANGLE_KI, BODANPAN_ANGLE_KD};
	PID_init(&bodanpan_angle_pid, PID_POSITION, motor_angle_pan_pid, M2006_MOTOR_ANGLE_PAN_PID_MAX_OUT, M2006_MOTOR_ANGLE_PAN_PID_MAX_IOUT);
		
	fp32 motor_speed_pan_pid[3] = {BODANPAN_SPEED_KP, BODANPAN_SPEED_KI, BODANPAN_SPEED_KD};
	PID_init(&bodanpan_speed_pid, PID_POSITION, motor_speed_pan_pid, M2006_MOTOR_SPEED_PAN_PID_MAX_OUT, M2006_MOTOR_SPEED_PAN_PID_MAX_IOUT);
}

void bodanpan_motor_control(void)
{
	bodanpan.code_now = motor_chassis[4].code;
	
	PID_calc(&bodanpan_angle_pid,bodanpan.code_now,bodanpan.code_set);
	
	bodanpan.speed_set = bodanpan_angle_pid.out/10.0f;
	bodanpan.speed=motor_chassis[4].speed_rpm/10.0f;
	
	PID_calc(&bodanpan_speed_pid,bodanpan.speed,bodanpan.speed_set);
	
	motor_chassis[4].given_current=bodanpan_speed_pid.out;
	
	CAN_cmd_pan(motor_chassis[4].given_current);

}

void IC_story(uint8_t data)
{
	bodanpan.IC_date_pan[ball_num] = data;
	bodanpan.box_state[ball_num] = 1;
	ball_num++;

}

