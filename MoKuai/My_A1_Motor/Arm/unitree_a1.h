#ifndef UNITREE_A1_H
#define UNITREE_A1_H

#include "main.h"
#include "string.h"
#include "A1_motor_msg.h"

#define Unitree_RX_BUF_NUM 78

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;

extern uint8_t Unitree_rx6_buf[][Unitree_RX_BUF_NUM];
extern ServoComdDataV3 motor_data_rx6;

//CRC校验
uint32_t crc32_core(uint32_t *ptr, uint32_t len);
// 电机串口初始化
void Unitree_Usart6_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque);

void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW);


// 修改控制参数
void ModifyData(motor_send_t *send, uint8_t ID, float Torque, float W, float POS, float KP, float KW);
// 发送电机控制命令
void UnitreeSend(motor_send_t *motor);
// 解包电机返回数据
void ExtractData(motor_recv_t *recv, uint8_t *rx_temp);
// 校验数据并复制接收数据到数据包
void data_rx_copy(uint8_t *motor_data_rx, uint8_t *data);
// 自制宇树转动
void unitree_move(uint8_t flag, float pos, float w);

#endif
