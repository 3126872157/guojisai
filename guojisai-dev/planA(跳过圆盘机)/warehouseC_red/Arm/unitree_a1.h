#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include "main.h"
#include "string.h"
#include "A1_motor_msg.h"

#define Unitree_RX_BUF_NUM 78

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define UNITREE_MOTOR_HUART huart1
#define UNITREE_MOTOR_UART USART1
#define UNITREE_MOTOR_HDMA_RX hdma_usart1_rx
#define UNITREE_MOTOR_HDMA_TX hdma_usart1_tx

// 电机串口初始化
void unitree_Uart_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
//力矩控制模式
void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque);
// 速度控制模式
void modfiy_speed_cmd(motor_send_t *send, uint8_t id, float Omega, float kw);
//位置控制模式
void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW);
//混合模式
void modfiy_mix_cmd(motor_send_t *send, uint8_t ID, float Torque, float POS, float W, float KP, float KW);
// 发送电机控制命令
void UnitreeSend(motor_send_t *motor);
// 解包电机返回数据
void ExtractData(motor_recv_t *recv, ServoComdDataV3 *rx_temp);
// 电机串口重新初始化
void unitree_Uart_RE_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);


#endif
