
//#ifndef __UNITREEA1_CMD__
//#define __UNITREEA1_CMD__

//#include "motor_msg.h"
//#include "usart.h"

//#define Unitree_RX_BUF_NUM 78

//extern motor_send_t cmd_motor;  	// 电机发送数据体
//extern motor_recv_t Date_motor;     // 电机接收数据体

///**
// @brief 对应电机参数修改
// @param send 为电机参数
// @param id   发送接收目标电机的id
// @param pos  为电机旋转圈数，1为一圈
// @param KP   电机刚度系数
// @param KW   电机速度系数
//*/
////位置模式
//void modfiy_position_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW);

//// 速度模式
//void modfiy_speed_cmd(motor_send_t *send,uint8_t id, float Omega);

//// 力矩模式
//void modfiy_torque_cmd(motor_send_t *send,uint8_t id, float torque);

////用来和电机通讯的代码，将获取的数据存入对应结构体中
//void unitreeA1_rxtx(UART_HandleTypeDef *huart);

//uint32_t crc32_core(uint32_t *ptr, uint32_t len);

//#endif
