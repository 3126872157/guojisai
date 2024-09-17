  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================    
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
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


bool_t chassis_code_reset_flag; //����4�������̼������־λ
extern bool_t can_reset_flag[5];//���������̼ƹ���ı�־(���ϲ����̵��һ��5��)

float slow_start_distance_k = 1.0f;

//���õ���4���������̼������־λ
void chassis_code_reset(void)
{
	for(uint8_t i=0;i<=3;i++)
		can_reset_flag[i]=1;
}

/**
  * @brief          ��������
  * @author         RM
  * @param[in]      chassis_move_vector��������
  * @param[in]    	������״̬Ҳ�ǲ��Ե�״̬��Ҫ���볢��ʲô���ܿ��Զ�����
  * @retval         ���ؿ�
  */
static void chassis_stop_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          ���̺������
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_vector��������
  * @param[in]		��������õ���ƽ��������ڲ�����x_set��y_set
  * @retval         ���ؿ�
  */
static void  chassis_infrared_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          ���̳���ģʽ
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_vector��������
	* @param[in]			�������PID�õ��������ǽǶȻ���PID������3�������ֱ��Ǵ���������PID�ĸ������ݣ��������������ܹ�ת�˶��ٽǶȣ�����������Ҫת���ĽǶȣ������������ݸ�ֵ��wz
									������Ϊ����˳ʱ�룬������ʱ�룩����Ϊʲô���������ܹ�ת�˶��ٶȣ���Ҫ����Ϊ���������ݴ����ֻ����-180��180������������ƾ���Զ�ﲻ��Ŀ���˱���360��
  * @retval         ���ؿ�
  */
static void chassis_ultrasonic_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

/**
  * @brief          ����ƽ��+��ת
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_and_rotate_vector��������
  * @param[in]		ƽ����+�����ǽǶȻ�����ڲ����ֱ�Ϊchassis->x_set,y_set,gyro_set�������������ɿ���������
  * @retval         ���ؿ�
  */
static void chassis_move_and_rotate_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);

static void chassis_V_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_vector);



//���⣬�������ģʽ����
//����ͨ��extern����������������ʱ�ı������Ϊģʽ�����ı����ģʽ
chassis_mode_e chassis_behaviour_mode = CHASSIS_MOVE_AND_ROTATE;



/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
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
		chassis_move_vector->chassis_mode = CHASSIS_INFRARED;		//��仰�����溯�����ƶ�������
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
  * @brief          ��������
  * @author         RM
  * @param[in]      chassis_move_vector��������
  * @param[in]    	������״̬Ҳ�ǲ��Ե�״̬��Ҫ���볢��ʲô���ܿ��Զ�����
  * @retval         ���ؿ�
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
  * @brief          ���̺������
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_vector��������
  * @param[in]		��������õ���ƽ��������ڲ�����x_set��y_set
  * @retval         ���ؿ�
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
		//�Ժ����ƣ���������������������������
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
//		*vx_set = -IR_V_x/2.5f;
//		*vy_set = IR_V_y/2.5f;
}

/**
  * @brief          ���̳���ģʽ
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_vector��������
  * @param[in]		�������PID�õ��������ǽǶȻ���PID������3�������ֱ��Ǵ���������PID�ĸ������ݣ��������������ܹ�ת�˶��ٽǶȣ�����������Ҫת���ĽǶȣ������������ݸ�ֵ��wz
					������Ϊ����˳ʱ�룬������ʱ�룩����Ϊʲô���������ܹ�ת�˶��ٶȣ���Ҫ����Ϊ���������ݴ����ֻ����-180��180������������ƾ���Զ�ﲻ��Ŀ���˱���360��
  * @retval         ���ؿ�
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
  * @brief          ����ƽ��+��ת
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ˳ʱ����ת����ֵ ��ʱ����ת
  * @param[in]      chassis_move_and_rotate_vector��������
  * @param[in]		ƽ����+�����ǽǶȻ�����ڲ����ֱ�Ϊchassis->x_set,y_set,gyro_set�������������ɿ���������
  * @retval         ���ؿ�
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
	//�˶�ʱ��ʹ�ýǶȻ�����ƫ���ǲ���
	if(chassis_move_vector->x_set||chassis_move_vector->y_set)
	{
	
		*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
		*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_move_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
		
	}
	//��ֹʱ��ʹ��ת����ת����
	else
	{
		
		*vx_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->x,chassis_move_vector->x_set);
		*vy_set = PID_calc(&chassis_move_vector->motor_distance_pid,chassis_move_vector->y,chassis_move_vector->y_set);
//		*wz_set = PID_calc(&chassis_move_vector->motor_gyro_pid,chassis_move_vector->gyro,chassis_move_vector->gyro_set);
		
	}
}

//�Ժ����ƣ�����������������
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
