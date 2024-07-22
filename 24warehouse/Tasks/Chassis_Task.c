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

//���Ա�����־λ��1Ϊ������ȫģʽ����Ϊ����dap�˳����Ե�ʱ���и��ʻ���
uint8_t safe_flag = 0;
//�����ã�����������������������
fp32 vx_set = 0.0f,
	 vy_set = 0.0f,
	 angle_set = 0.0f;

//�����˶�����
chassis_move_t chassis_move;

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

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
		if(safe_flag == 0 )
		{
			CAN_cmd_chassis(0, 0, 0, 0);
		}
		else
		{
			//�������ݸ���
			chassis_feedback_update(&chassis_move);
		
			//���̿���������
			chassis_set_contorl(&chassis_move);
				
			//���̿���PID����
			chassis_control_loop(&chassis_move);
		
			//���Ϳ��Ƶ���
			CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current,
							chassis_move.motor_chassis[1].give_current,
							chassis_move.motor_chassis[2].give_current,
							chassis_move.motor_chassis[3].give_current);
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
    const static fp32 motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

	//��ȡ���̵������ָ�룬��ʼ��PID 
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M2006_MOTOR_SPEED_PID_MAX_OUT, M2006_MOTOR_SPEED_PID_MAX_IOUT);
    }
	
	//�Ժ����ƣ���������������������
	//ƽ��������̻���
	
	//�ǶȻ�
	
	
    //���̽Ƕ�pidֵ
    //const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    //��ʼ���Ƕ�PID
    //PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //���̿���״̬Ϊֹͣ״̬
    //chassis_move_init->chassis_mode = CHASSIS_STOP;
    
    //��ȡ��������̬��ָ��
    //chassis_move_init->chassis_INS_angle = get_INS_angle_point();
   
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

    for (uint8_t i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ��ע�������ţ���������������ʱ��ֱ���0��1��2��3�ŵ����wz����Ϊ��ʱ�룬���Ǻʹ����ִ���һ��
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

	//������̼ƣ�ת�ӽǶȼƣ������Ǹ��£�pid����
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

	//�Ժ����ƣ�����������������
    //fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //��ȡ������������ֵ
    //chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

	fp32 wz_set = angle_set;
	
	chassis_move_control->vx_set = vx_set;
	chassis_move_control->vy_set = vy_set;
	chassis_move_control->wz_set = wz_set;
	
    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
	chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
	chassis_move_control->wz_set = fp32_constrain(chassis_move_control->wz_set, chassis_move_control->wz_min_speed, chassis_move_control->wz_max_speed);
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
    }

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
    }
}
