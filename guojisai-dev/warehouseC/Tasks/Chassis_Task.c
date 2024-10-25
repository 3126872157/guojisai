/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_task.h"
#include "CAN_receive.h"
#include "pid.h"
#include "arm_math.h"
#include "INS_task.h"
#include "chassis_behaviour.h"

#define abs(x)	( (x>0) ? (x) : (-x) )

//���Ա�����־λ��1Ϊ������ȫģʽ����Ϊ����dap�˳����Ե�ʱ���и��ʻ���
uint8_t safe_flag = 1;

extern float send_data[10];
extern float rx_gyro;//���յ��������ǽǶ���Ϣ
extern bool_t chassis_code_reset_flag;//������̼������־λ
extern motor_measure_t motor_chassis[5];
extern float ramp_x;
extern float ramp_y;
extern float ramp_z;

extern fp32 V_mode_x_speed;
extern fp32 V_mode_y_speed;
extern fp32 V_mode_w_speed;

//fp32 limit_xspeed_set=0;
//fp32 limit_yspeed_set=0;
//fp32 limit_wspeed_set=0;
//uint8_t cntx=0;
//uint8_t cnty=0;
//uint8_t cntw=0;


//����������
extern fp32 my_angle[3];
extern fp32 my_yaw_offset;

//�����ã�����������������������
//fp32 vx_set = 0.0f,
//vy_set = 0.0f,
//angle_set = 0.0f;
#include "vofa.h"
extern float vofa_send_data[10];
float test_speed_set = 0;


//�����˶�����
chassis_move_t chassis_move;

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//���̲�������
void chassis_reset(chassis_move_t *chassis_move_reset)
{
	if(chassis_code_reset_flag == 1)
	{
		for(int i = 0;i < 4;i++)
		{
			//����������
			motor_chassis[i].code = 0;
			motor_chassis[i].round_cnt = 0;
			motor_chassis[i].offset_code = motor_chassis[i].ecd;
			
			//����ٶ�pid
			PID_clear(&chassis_move_reset->motor_speed_pid[i]);
		}
		//���λ��pid
		PID_clear(&chassis_move_reset->motor_distance_pid);
		
		//���Ŀ��ֵ
		chassis_move_reset->x_set = 0;
		chassis_move_reset->y_set = 0;
		chassis_move_reset->chassis_mode = CHASSIS_STOP;
		
		//�������ֵ
		chassis_move_reset->x = 0;
		chassis_move_reset->y = 0;
		
		V_mode_x_speed = 0;
		V_mode_y_speed = 0;
		V_mode_w_speed = 0;
		
		ramp_x = 0;
		ramp_y = 0;
		ramp_z = 0;
		
		chassis_move_reset->vx_max_speed = 50.0f;
		chassis_move_reset->vx_min_speed = -50.0f;
		chassis_move_reset->vy_max_speed = 50.0f;
		chassis_move_reset->vy_min_speed = -50.0f;
		
		chassis_code_reset_flag = 0;
	}
}


//���̿�������
void chassis_task(void const * argument)
{
	//����һ��ʱ��
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
	
	//���̳�ʼ��
    chassis_init(&chassis_move);
	
	//�жϵ��̵���Ƿ�����
	//����Ͳ�ʵ����
	
	while(1)
	{
		if(safe_flag == 1 )
		{
			CAN_cmd_chassis(0, 0, 0, 0);
			CAN_cmd_pan(0);
		}
		else
		{
			//�������ݸ���
			chassis_feedback_update(&chassis_move);
		
			//���̿���������
			chassis_set_contorl(&chassis_move);
				
			//���̿���PID����
			chassis_control_loop(&chassis_move);
			
			//�����̵������
			bodanpan_motor_control();
			
			//���͵��̿��Ƶ���
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,
							chassis_move.motor_chassis[1].give_current,
							chassis_move.motor_chassis[2].give_current,
							chassis_move.motor_chassis[3].give_current);
			
			//����pid����
//			vofa_send_data[0] = chassis_move.motor_chassis[0].speed_set;
//			vofa_send_data[1] = chassis_move.motor_chassis[0].speed;
//			vofa_send_data[2] = chassis_move.motor_chassis[1].speed;
//			vofa_send_data[3] = chassis_move.motor_chassis[2].speed;
//			vofa_send_data[4] = chassis_move.motor_chassis[3].speed;
//			
//			my_vofa_printf(5);
		}
		//ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
	}
}

/**
  * @brief          ��ʼ��"chassis_move"����������pid��ʼ����2006���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
  * @param[out]     chassis_move_init:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
	
	//�ٶȻ�
    const static fp32 motor_speed_pid[4][3] = {{M2006_MOTOR1_SPEED_PID_KP, M2006_MOTOR1_SPEED_PID_KI, M2006_MOTOR1_SPEED_PID_KD},
											   {M2006_MOTOR2_SPEED_PID_KP, M2006_MOTOR2_SPEED_PID_KI, M2006_MOTOR2_SPEED_PID_KD},
											   {M2006_MOTOR3_SPEED_PID_KP, M2006_MOTOR3_SPEED_PID_KI, M2006_MOTOR3_SPEED_PID_KD},
											   {M2006_MOTOR4_SPEED_PID_KP, M2006_MOTOR4_SPEED_PID_KI, M2006_MOTOR4_SPEED_PID_KD}};

	//��ȡ���̵������ָ�룬��ʼ��PID 
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid[i], M2006_MOTOR1_SPEED_PID_MAX_OUT, M2006_MOTOR1_SPEED_PID_MAX_IOUT);
    }
	
	//ƽ��������̻���
	const static fp32 motor_distance_pid[3] = {M2006_MOTOR_DISTANCE_PID_KP, M2006_MOTOR_DISTANCE_PID_KI, M2006_MOTOR_DISTANCE_PID_KD};
	PID_init(&chassis_move_init->motor_distance_pid, PID_POSITION, motor_distance_pid, M2006_MOTOR_DISTANCE_PID_MAX_OUT, M2006_MOTOR_DISTANCE_PID_MAX_IOUT);
	
	//ת��
	const static fp32 motor_gyro_pid[3] = {M2006_MOTOR_GYRO_PID_KP, M2006_MOTOR_GYRO_PID_KI, M2006_MOTOR_GYRO_PID_KD};
	PID_init(&chassis_move_init->motor_gyro_pid, PID_POSITION, motor_gyro_pid, M2006_MOTOR_GYRO_PID_MAX_OUT, M2006_MOTOR_GYRO_PID_MAX_IOUT);
	
    //�ǶȻ�
    const static fp32 motor_move_gyro_pid[3] = {M2006_MOTOR_MOVE_GYRO_PID_KP, M2006_MOTOR_MOVE_GYRO_PID_KI, M2006_MOTOR_MOVE_GYRO_PID_KD};
	PID_init(&chassis_move_init->motor_move_gyro_pid, PID_POSITION, motor_move_gyro_pid, M2006_MOTOR_MOVE_GYRO_PID_MAX_OUT, M2006_MOTOR_MOVE_GYRO_PID_MAX_IOUT);
	
	//��ȡ��������̬��ָ��
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	
	//������������0
	chassis_move_init->gyro=0.0f;
	chassis_move_init->gyro_set=0.0f;
	chassis_move_init->last_gyro=0.0f;
	
    //���̿���״̬Ϊֹͣ״̬
    chassis_move_init->chassis_mode = CHASSIS_STOP;
	
	//������Ҳ���Գ�ʼ��
   
    //���������� ��С�ٶ�
    chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	chassis_move_init->wz_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Z;
    chassis_move_init->wz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ���õ���ģʽ
  * @param[in]		chassis_mode����ģʽ�������chassis_task.h����
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode, chassis_mode_e chassis_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_move_mode->chassis_mode = chassis_mode;
}

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
	
	//�������Ƿ���Ҫ����
	chassis_reset(chassis_move_update);

    for (uint8_t i = 0; i < 4; i++)
    {
		//ת�ӽǶȼ�
//		fp32 temp_angle = chassis_move_update->motor_chassis[i].chassis_motor_measure->ecd - chassis_move_update->motor_chassis[i].chassis_motor_measure->last_ecd;
//		if(temp_angle >= 4096)
//		{
//			temp_angle = (8192.0f - chassis_move_update->motor_chassis[1].chassis_motor_measure->ecd) + chassis_move_update->motor_chassis[1].chassis_motor_measure->last_ecd;
//		}
//		else if(temp_angle <= -4096)
//		{
//			temp_angle = (8192.0f - chassis_move_update->motor_chassis[1].chassis_motor_measure->last_ecd) + chassis_move_update->motor_chassis[1].chassis_motor_measure->ecd;
//		}
		
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
		
		//chassis_move_update->motor_chassis[i].total_angle +=(temp_angle)/8192.0f*360.0f;
    }

    //�ٶȸ��£����µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ��ע�������ţ���������������ʱ��ֱ���0��1��2��3�ŵ����wz����Ϊ��ʱ�룬���Ǻʹ����ִ���һ��
    chassis_move_update->vx = ( - chassis_move_update->motor_chassis[0].speed
								+ chassis_move_update->motor_chassis[1].speed
								+ chassis_move_update->motor_chassis[2].speed
								- chassis_move_update->motor_chassis[3].speed)
								* MOTOR_SPEED_TO_CHASSIS_SPEED_VX
								* OMNI_WHEEL_SPEED_COMPOSITION;
    chassis_move_update->vy = ( - chassis_move_update->motor_chassis[0].speed
								- chassis_move_update->motor_chassis[1].speed
								+ chassis_move_update->motor_chassis[2].speed
								+ chassis_move_update->motor_chassis[3].speed)
								* MOTOR_SPEED_TO_CHASSIS_SPEED_VY
								* OMNI_WHEEL_SPEED_COMPOSITION;
    chassis_move_update->wz = ( - chassis_move_update->motor_chassis[0].speed
								- chassis_move_update->motor_chassis[1].speed
								- chassis_move_update->motor_chassis[2].speed
								- chassis_move_update->motor_chassis[3].speed)
								* MOTOR_SPEED_TO_CHASSIS_SPEED_WZ;
	
	//λ�ø��£�ͬ��Ҫ�˶�ѧ����
	chassis_move_update->x = ( - chassis_move_update->motor_chassis[0].chassis_motor_measure->code
							   + chassis_move_update->motor_chassis[1].chassis_motor_measure->code
							   + chassis_move_update->motor_chassis[2].chassis_motor_measure->code
							   - chassis_move_update->motor_chassis[3].chassis_motor_measure->code)
							   * OMNI_WHEEL_SPEED_COMPOSITION / 4.0f
							   * M2006_MOTOR_ECD_TO_DISTANCE;
	chassis_move_update->y = ( - chassis_move_update->motor_chassis[0].chassis_motor_measure->code
							   - chassis_move_update->motor_chassis[1].chassis_motor_measure->code
							   + chassis_move_update->motor_chassis[2].chassis_motor_measure->code
							   + chassis_move_update->motor_chassis[3].chassis_motor_measure->code)
							   * OMNI_WHEEL_SPEED_COMPOSITION / 4.0f
							   * M2006_MOTOR_ECD_TO_DISTANCE;
	
	//��������������
	chassis_move_update->last_gyro = chassis_move_update->gyro;
	chassis_move_update->gyro = (*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - my_yaw_offset) * R2D;
	
	//ת�ӽǶȼƣ�pid���º������Ժ�����
	
}

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;
    //��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
	
	//�Ժ�������������ģʽ
	if(chassis_move_control->chassis_mode == CHASSIS_STOP)
	{
		safe_flag = 1;
	}
	else if(chassis_move_control->chassis_mode == CHASSIS_MOVE_AND_ROTATE)
	{
		safe_flag = 0;
		
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = wz_set;
		
		chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
	}
	else if(chassis_move_control->chassis_mode == CHASSIS_V)
	{
		safe_flag = 0;
		
		chassis_move_control->vx_set = vx_set;
		chassis_move_control->vy_set = vy_set;
		chassis_move_control->wz_set = wz_set;
		
		chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
		chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
		chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
	}
}

/**
  * @brief          �ĸ�ȫ�������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ�ȫ�����ٶ�
  * @retval         none
  */
static void chassis_vector_to_omni_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    wheel_speed[0] = (- vx_set - vy_set) * OMNI_WHEEL_SPEED_COMPOSITION - wz_set;
    wheel_speed[1] = (+ vx_set - vy_set) * OMNI_WHEEL_SPEED_COMPOSITION - wz_set;
    wheel_speed[2] = (+ vx_set + vy_set) * OMNI_WHEEL_SPEED_COMPOSITION - wz_set;
    wheel_speed[3] = (- vx_set + vy_set) * OMNI_WHEEL_SPEED_COMPOSITION - wz_set;
}

float error_now;
/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f;
	fp32 vector_rate = 0.0f;	//���ٱ�
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
	fp32 error_offset = 0.0f;

    //ȫ�����˶��ֽ�
    chassis_vector_to_omni_wheel_speed(chassis_move_control_loop->vx_set,
                                       chassis_move_control_loop->vy_set,
									   chassis_move_control_loop->wz_set,
									   wheel_speed);

    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        if(chassis_move_control_loop->motor_chassis[i].speed_set >= 0) temp =   chassis_move_control_loop->motor_chassis[i].speed_set;
		if(chassis_move_control_loop->motor_chassis[i].speed_set <  0) temp = - chassis_move_control_loop->motor_chassis[i].speed_set;
        if (max_vector < temp)
        {
            max_vector = temp;
        }
		
		//����ĸ�����error�ܺ�
		error_offset += chassis_move_control_loop->motor_speed_pid[i].error[0];
    }
	
	//���ƽ��errorֵ
	error_offset /= 4;

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //����pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
		//����ÿ�������error����������С
//		chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out) * chassis_move_control_loop->motor_speed_pid[i].error[0] / error_offset;
    }
}
