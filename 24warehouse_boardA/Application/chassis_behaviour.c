  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================    
    如果要添加一个新的行为模式
    1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"chassis_behaviour_mode_set"这个函数中，添加新的逻辑判断，给chassis_behaviour_mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"

#define fabs(x) ((x) >= 0 ? (x) : -(x))


bool_t chassis_code_reset_flag; //底盘4个电机里程计清零标志位
extern bool_t can_reset_flag[5];//单个电机里程计归零的标志(加上拨蛋盘电机一共5个)

float slow_start_distance_k = 1.0f;

//设置底盘4个电机的里程计清零标志位
void chassis_code_reset(void)
{
	for(uint8_t i=0;i<=3;i++)
		can_reset_flag[i]=1;
}

/**
  * @brief          底盘无力
  * @author         RM
  * @param[in]      chassis_move_vector底盘数据
  * @param[in]    	无力的状态也是测试的状态，要是想尝试什么功能可以丢这里
  * @retval         返回空
  */
static void chassis_stop_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          底盘红外控制
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_vector底盘数据
  * @param[in]		这个函数用的是平动环，入口参数是x_set和y_set
  * @retval         返回空
  */
static void  chassis_infrared_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          底盘超声模式
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_vector底盘数据
	* @param[in]			这个函数PID用的是陀螺仪角度环，PID函数的3个参数分别是传入陀螺仪PID的各项数据，传入由陀螺仪总共转了多少角度，传入我们想要转到的角度，解算完后把数据赋值给wz
									（数据为正则顺时针，负则逆时针）至于为什么是陀螺仪总共转了多少度，主要是因为陀螺仪数据处理后只能是-180到180，超过这个限制就永远达不到目标了比如360度
  * @retval         返回空
  */
static void chassis_ultrasonic_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          底盘平动+旋转
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_and_rotate_vector底盘数据
  * @param[in]		平动环+陀螺仪角度环，入口参数分别为chassis->x_set,y_set,gyro_set，改这三个即可控制三个环
  * @retval         返回空
  */
static void chassis_move_and_rotate_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

static void chassis_V_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);



//留意，这个底盘模式变量
//可以通过extern来在其他任务里随时改变底盘行为模式，并改变底盘模式
chassis_mode_e chassis_behaviour_mode = CHASSIS_MOVE_AND_ROTATE;



/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */


void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
	
	if (chassis_behaviour_mode == CHASSIS_INFRARED)
    {
        chassis_infrared_control(vx_set, vy_set, wz_set, chassis_move_vector);
		chassis_move_vector->chassis_mode = CHASSIS_INFRARED;		//这句话从上面函数里移动到这里
    }
    else if (chassis_behaviour_mode == CHASSIS_ULTRASONIC)
	{
        chassis_ultrasonic_control(vx_set, vy_set, wz_set, chassis_move_vector);
		chassis_move_vector->chassis_mode = CHASSIS_ULTRASONIC;
    }
	else if (chassis_behaviour_mode == CHASSIS_MOVE_AND_ROTATE)
    {
        chassis_move_and_rotate_control(vx_set, vy_set, wz_set, chassis_move_vector);
		chassis_move_vector->chassis_mode = CHASSIS_MOVE_AND_ROTATE;
    }
    else if (chassis_behaviour_mode == CHASSIS_STOP)
    {
        chassis_stop_control(vx_set, vy_set, wz_set, chassis_move_vector);
		chassis_move_vector->chassis_mode = CHASSIS_STOP;
    }
	else if (chassis_behaviour_mode == CHASSIS_V)
    {
        chassis_V_control(vx_set, vy_set, wz_set, chassis_move_vector);
		chassis_move_vector->chassis_mode = CHASSIS_V;
    }
}

/**
  * @brief          底盘无力
  * @author         RM
  * @param[in]      chassis_move_vector底盘数据
  * @param[in]    	无力的状态也是测试的状态，要是想尝试什么功能可以丢这里
  * @retval         返回空
  */
static void chassis_stop_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
	*vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘红外控制
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_vector底盘数据
  * @param[in]		这个函数用的是平动环，入口参数是x_set和y_set
  * @retval         返回空
  */
bool_t flag_infrared=0;
static void  chassis_infrared_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
			if(chassis_code_reset_flag==1)
		{
			chassis_code_reset();
			
			chassis_move_vector->x_set=0.0f;
			chassis_move_vector->y_set=0.0f;
			chassis_code_reset_flag=0;
		}
		//稍后完善！！！！！！！！！！！！！！
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
//		*vx_set = -IR_V_x/2.5f;
//		*vy_set = IR_V_y/2.5f;
}

/**
  * @brief          底盘超声模式
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_vector底盘数据
  * @param[in]		这个函数PID用的是陀螺仪角度环，PID函数的3个参数分别是传入陀螺仪PID的各项数据，传入由陀螺仪总共转了多少角度，传入我们想要转到的角度，解算完后把数据赋值给wz
					（数据为正则顺时针，负则逆时针）至于为什么是陀螺仪总共转了多少度，主要是因为陀螺仪数据处理后只能是-180到180，超过这个限制就永远达不到目标了比如360度
  * @retval         返回空
  */
bool_t flag_ultrasonic=0;
static void chassis_ultrasonic_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
	
	if(chassis_code_reset_flag==1)
	{
		chassis_code_reset();

		chassis_move_vector->x_set=0.0f;
		chassis_move_vector->y_set=0.0f;
		chassis_code_reset_flag=0;
	}
	
	if(fabs(chassis_move_vector->y-30.0f)>0.5f)
	{
		osDelay(50);
		if(fabs(chassis_move_vector->y-30.0f)>0.5f)
		{
			*vy_set=5.0f;
		}
	}
	else
	{
		*vy_set=0.0f;
	}
	if(flag_ultrasonic==1 && fabs(chassis_move_vector->x-50.0f)>0.5f)
	{
		osDelay(50);
		if(fabs(chassis_move_vector->x-50.0f)>0.5f)
		{
		*vx_set=8.0f;
		}
	}
	else
	{
		*vx_set=0.0f;
	}
	*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
}

/**
  * @brief          底盘平动+旋转
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 顺时针旋转，负值 逆时针旋转
  * @param[in]      chassis_move_and_rotate_vector底盘数据
  * @param[in]		平动环+陀螺仪角度环，入口参数分别为chassis->x_set,y_set,gyro_set，改这三个即可控制三个环
  * @retval         返回空
  */
static void chassis_move_and_rotate_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
	if(chassis_code_reset_flag==1)
	{
		chassis_code_reset();

		chassis_move_vector->x_set=0.0f;
		chassis_move_vector->y_set=0.0f;
		chassis_code_reset_flag=0;
	}
//	float x_set = 0.0f,y_set = 0.0f;
	//运动时，使用角度环保持偏航角不变
	if(chassis_move_vector->x_set||chassis_move_vector->y_set)
	{
	
		*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
		*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_move_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
		
	}
	//静止时，使用转向环旋转底盘
	else
	{
		
		*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
		*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
		
	}
}

//稍后完善！！！！！！！！！
static void chassis_V_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
{
	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
    {
        return;
    }
	//*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
	//*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
	//*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
}

//void chassis_vx_movey_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
//{
//	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
//    {
//        return;
//		//*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
//		*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
//		chassis_move_vector->chassis_mode=CHASSIS_V;
//    }
//}

//void chassis_vy_movex_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector)
//{
//	if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_vector == NULL)
//    {
//        return;
//		*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
//		//*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
//		chassis_move_vector->chassis_mode=CHASSIS_V;
//    }
//}
