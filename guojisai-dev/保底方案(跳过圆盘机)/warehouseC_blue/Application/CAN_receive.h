/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_2006_M1_ID = 0x201,
    CAN_2006_M2_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
    CAN_2006_M4_ID = 0x204,
	
	//�������
    CAN_TRIGGER_MOTOR_ID = 0x205,
    CAN_TRIGGER_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;			//��е�Ƕȣ���Χ��0-8191����Ӧ0-360��
    volatile int16_t speed_rpm;      //���ת��(r per minute)
    int16_t given_current;  //ת�ص���
    uint8_t temperate;      //����¶�
    int16_t last_ecd;
	fp32 code;				//��Ϊ��̼ƣ���¼ת�Ӵӿ�������ǰ���ܻ�е�Ƕ�
	uint16_t offset_code;	//��Ϊ�ϵ�ʱ�ĳ�ʼƫ�ƽǶ�
	int32_t round_cnt;		//ת��Ȧ��
							//���㹫ʽΪ��code = round_cnt * 8192 + ecd - offset_code 
} motor_measure_t;

/**
  * @brief          ���͵�����Ƶ���(0x205)�����Ʋ����̵��
  * @param[in]      motor5: (0x205) 2006������Ƶ���,
  * @retval         none
  */
extern void CAN_cmd_pan(int16_t motor_trigger);

/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor2: (0x202) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor3: (0x203) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor4: (0x204) 2006������Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
