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

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"

extern CAN_HandleTypeDef hcan1;

//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
#define abs(x)	( (x>0) ? (x) : (-x) )
	
/*
	������ݣ�
		0:���̵��1 2006���,
		1:���̵��2 2006���,
		2:���̵��3 2006���,
		3:���̵��4 2006���;
		4:ת�̵�� 2006���;
*/
motor_measure_t motor_chassis[5];
uint8_t start_flag[5] = {0};			//������flag���������ÿ�����codeֵ
bool_t can_reset_flag[5]={0};			//��̼ƹ���ı�־

static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_2006_M1_ID:
        case CAN_2006_M2_ID:
        case CAN_2006_M3_ID:
        case CAN_2006_M4_ID:
			
        case CAN_TRIGGER_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_2006_M1_ID;
            get_motor_measure(&motor_chassis[i], rx_data);
			
			//��̼�
			if(start_flag[i] != 0)
			{
				//����һ
				if(motor_chassis[i].ecd - motor_chassis[i].last_ecd > 4096)
					motor_chassis[i].round_cnt --;
				else if(motor_chassis[i].ecd - motor_chassis[i].last_ecd < -4096)
					motor_chassis[i].round_cnt ++;
				motor_chassis[i].code = motor_chassis[i].round_cnt * 8192 + motor_chassis[i].ecd - motor_chassis[i].offset_code;
				
				//������
//				int16_t temp1 = motor_chassis[i].ecd - motor_chassis[i].last_ecd;
//				int16_t temp2 = temp1 + (temp1 < 0 ? 8192 : -8192);
//				motor_chassis[i].code += abs(temp2) < abs(temp1) ? temp2 : temp1;
				
				//����codeֵʱ
				if(can_reset_flag[i] != 0 )
				{
					motor_chassis[i].code = 0;
					motor_chassis[i].round_cnt = 0;
					motor_chassis[i].offset_code = motor_chassis[i].ecd;
					can_reset_flag[i] = 0;
				}
			}
			else
			{
				start_flag[i] = 1;
				motor_chassis[i].code = 0;
				motor_chassis[i].round_cnt = 0;
				motor_chassis[i].offset_code = motor_chassis[i].ecd;
			}
			break;
        }

        default:
        {
            break;
        }
    }
}

/**
  * @brief          ���͵�����Ƶ���(0x205)�����Ʋ����̵��
  * @param[in]      motor5: (0x205) 2006������Ƶ���,
  * @retval         none
  */
void CAN_cmd_pan(int16_t motor_trigger)
{
	uint8_t TX_Data[2];
	uint32_t send_mail_box;
	CAN_TxHeaderTypeDef Tx_Msg;

	Tx_Msg.StdId = CAN_TRIGGER_ALL_ID;	//��5-8�ŵ�����ͱ���ʱ�õı�ʶ��
	Tx_Msg.IDE = CAN_ID_STD;
	Tx_Msg.RTR = CAN_RTR_DATA;
	Tx_Msg.DLC = 0x08;
  
	TX_Data[0] = motor_trigger >> 8;
	TX_Data[1] = motor_trigger;
	HAL_CAN_AddTxMessage(&hcan1, &Tx_Msg, TX_Data, &send_mail_box);
}

/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor2: (0x202) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor3: (0x203) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      motor4: (0x204) 2006������Ƶ���, ��Χ [-10000,10000]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          ���ص��̵�� 2006�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x07)];		//���7�������
}
