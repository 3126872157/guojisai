/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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
	
	//拨弹电机
    CAN_TRIGGER_MOTOR_ID = 0x205,
    CAN_TRIGGER_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;			//机械角度，范围是0-8191，对应0-360度
    volatile int16_t speed_rpm;      //电机转速(r per minute)
    int16_t given_current;  //转矩电流
    uint8_t temperate;      //电机温度
    int16_t last_ecd;
	fp32 code;				//此为里程计，记录转子从开机到当前的总机械角度
	uint16_t offset_code;	//此为上电时的初始偏移角度
	int32_t round_cnt;		//转动圈数
							//计算公式为：code = round_cnt * 8192 + ecd - offset_code 
} motor_measure_t;

/**
  * @brief          发送电机控制电流(0x205)，控制拨弹盘电机
  * @param[in]      motor5: (0x205) 2006电机控制电流,
  * @retval         none
  */
extern void CAN_cmd_pan(int16_t motor_trigger);

/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor2: (0x202) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor3: (0x203) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      motor4: (0x204) 2006电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);


#endif
