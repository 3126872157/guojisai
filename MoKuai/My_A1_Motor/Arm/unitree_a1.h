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

//CRCУ��
uint32_t crc32_core(uint32_t *ptr, uint32_t len);
// ������ڳ�ʼ��
void Unitree_Usart6_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque);

void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW);


// �޸Ŀ��Ʋ���
void ModifyData(motor_send_t *send, uint8_t ID, float Torque, float W, float POS, float KP, float KW);
// ���͵����������
void UnitreeSend(motor_send_t *motor);
// ��������������
void ExtractData(motor_recv_t *recv, uint8_t *rx_temp);
// У�����ݲ����ƽ������ݵ����ݰ�
void data_rx_copy(uint8_t *motor_data_rx, uint8_t *data);
// ��������ת��
void unitree_move(uint8_t flag, float pos, float w);

#endif
