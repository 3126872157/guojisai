#include "math.h"
#include "string.h"
#include "arm_ctrl.h"
#include "A1_motor_msg.h"

#define PI 3.1415926f
#define MAX_TORQUE 2.0f

uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM]; // dma���ջ���
ServoComdDataV3 motor_rx_temp;
extern unitree_ctrl_t unitree_Data;

// CRCУ��
uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++)
    {
        xbit = (uint32_t)1 << 31;
        data = ptr[i];

        for (uint32_t bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;

            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }

    return CRC32;
}

// У�����ݲ��ѽ���buf���Ƶ�����������
void data_rx_copy(ServoComdDataV3 *motor_data_rx, uint8_t *data)
{
    uint32_t CRC_data = *((uint32_t *)(data + 74));

    if (CRC_data == crc32_core((uint32_t *)data, 18))
    {
        memcpy(motor_data_rx, data, Unitree_RX_BUF_NUM);
    }
}

// ����A1������ڳ�ʼ��
void unitree_Usart6_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // ʹ��DMA���ڽ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    // ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while (hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    // �ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // �ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // ���ݳ���
    hdma_usart6_rx.Instance->NDTR = 2 * dma_buf_num;
    // ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    // ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

// �����ж�ִ��˫���彻����ƹ�һ��壩����������it.h
void USER_UART6_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart6)
    {
        static uint16_t this_time_rx_len = 0;

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // ʧ��DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            // ��ȡ�������ݳ��� ���� = �趨���� - ʣ�೤�ȣ�NDTR��
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            // �����趨ʣ�����ݳ���
            hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // �趨������1
            DMA2_Stream1->CR |= DMA_SxCR_CT;
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);
            // ��ʵ�ʽ��ճ��������۽��ܳ���һ��
            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, Unitree_rx6_buf[0]);
            }
        }
        else
        {
            // ʧ��DMA
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            // ͬ��
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
            // �����趨���ݳ���
            hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // �趨������0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            // ʹ��DMA
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, Unitree_rx6_buf[1]);
            }
        }
    }
}

// �����жϺ���������485תttlģ��״̬
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    static int unitree_cnt = 0;
    // ������ɽ���tx�жϻص�������RE��ƽ��ʹ��485תttlģ�����״̬
    HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_RESET);
    unitree_cnt++;
}

// ���͵��ָ��
void UnitreeSend(motor_send_t *motor)
{
    // ����DE��ƽ��ʹ��485תttlģ��ķ���ģʽ��ʧ�ܽ���ģʽ����ģ�鲻��ͬʱ��������ģʽ��
    HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_SET);
	//���ﲻ��Ҫ��ʱ
    while (HAL_UART_Transmit_IT(&huart6, (uint8_t *)&(motor->motor_send_data), 34) != HAL_OK);
}

// ��������������
void ExtractData(motor_recv_t *recv, ServoComdDataV3 *rx_temp)
{
    recv->motor_id = rx_temp->head.motorID;
    recv->mode = rx_temp->Mdata.mode;
    recv->Temp = rx_temp->Mdata.Temp;
    recv->MError = rx_temp->Mdata.MError;
    recv->T = ((float)rx_temp->Mdata.T) / 256;
    recv->W = ((float)rx_temp->Mdata.W) / 128 / 9.1f;
    recv->LW = rx_temp->Mdata.LW / 9.1f;

    recv->Acc = ((float)rx_temp->Mdata.Acc) / 128 / 9.1f;
    recv->Pos = ((float)rx_temp->Mdata.Pos) / 9.1f / 16384 * 2 * PI;

    recv->gyro[0] = ((float)rx_temp->Mdata.gyro[0]) * 0.00107993176f;
    recv->gyro[1] = ((float)rx_temp->Mdata.gyro[1]) * 0.00107993176f;
    recv->gyro[2] = ((float)rx_temp->Mdata.gyro[2]) * 0.00107993176f;

    recv->acc[0] = ((float)rx_temp->Mdata.acc[0]) * 0.0023911132f;
    recv->acc[1] = ((float)rx_temp->Mdata.acc[1]) * 0.0023911132f;
    recv->acc[2] = ((float)rx_temp->Mdata.acc[2]) * 0.0023911132f;
}

// ��������������
void WriteData(motor_send_t *write)
{
    write->motor_send_data.head.start[0] = 0xFE;
    write->motor_send_data.head.start[1] = 0xEE;
    write->motor_send_data.head.motorID = write->id;
    write->motor_send_data.head.reserved = 0x00;

    write->motor_send_data.Mdata.mode = write->mode;
    write->motor_send_data.Mdata.ModifyBit = 0xFF;
    write->motor_send_data.Mdata.ReadBit = 0x00;
    write->motor_send_data.Mdata.reserved = 0x00;
    write->motor_send_data.Mdata.Modify = 0;
    write->motor_send_data.Mdata.T = write->T * 256;
    write->motor_send_data.Mdata.W = write->W * 128;
    write->motor_send_data.Mdata.Pos = (int)((write->Pos / 6.2832f) * 16384.0f);
    write->motor_send_data.Mdata.K_P = (int16_t)(write->K_P * 2048);
    write->motor_send_data.Mdata.K_W = (int16_t)(write->K_W * 1024);

    write->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    write->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    write->motor_send_data.Mdata.Res = write->Res;

    write->motor_send_data.CRCdata = crc32_core((uint32_t *)(&(write->motor_send_data)), 7);
}

// ���ؿ���ģʽ��һ��������ģʽ����pid
void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque)
{
	//�޷�2N��M
	if(torque >= MAX_TORQUE)
	{
		torque = MAX_TORQUE;
	}
	else if(torque <= -MAX_TORQUE)
	{
		torque = -MAX_TORQUE;
	}
	
    // �û����õ�����
    send->id = id;
    send->mode = 10;
    send->Pos = 0.0;
    send->W = 0.0;
    send->T = torque;
    send->K_P = 0.0;
    send->K_W = 0.0;

    WriteData(send);
}

// λ�ÿ���ģʽ�����ģʽ�ܼ��ߣ�ƫ��̫������ǣ�ת��180�Ͳ���
void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW)
{
    send->id = id;
    send->mode = 10;
    send->Pos = 9.1f * Pos; // ������
    send->W = 0;
    send->T = 0.0;
    send->K_P = KP; // �ٷ��Ƽ�0.02��ʵ����Сһ����
    send->K_W = KW; // �ٷ��Ƽ�3.0��ʵ��1.0��һ�»�ȽϺ�

    WriteData(send);
}

// �ٶȿ���ģʽ
void modfiy_speed_cmd(motor_send_t *send, uint8_t id, float Omega)
{

    send->id = id;
    send->mode = 10;
    send->Pos = 0;
    send->W = Omega;
    send->T = 0.0;
    send->K_P = 0.0;
    send->K_W = 3.0;

    WriteData(send);
}

// ���ģʽ��ɶ���У���ʽ��Output = K_P * delta_Pos + K_W * delta_W + T��
void modfiy_mix_cmd(motor_send_t *send, uint8_t ID, float Torque, float POS, float W, float KP, float KW)
{
    send->id = ID;
    send->mode = 10; // Ĭ�ϻ��ģʽ
    send->T = Torque;
    send->W = W;
    send->Pos = 9.1f * POS; // ������
    send->K_P = KP;
    send->K_W = KW;
   
	WriteData(send);
}

//void unitree_move(uint8_t flag, float pos, float w)
//{
//    if (flag == 1) // ����
//    {
//        if ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) > pos)
//            return;
//        while ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) < (pos - 0.08))
//        {
////            ModifyData(&unitree_Data.unitree_send, 0, 0, 1.2, 0, 0, 3);
//            UnitreeSend(&unitree_Data.unitree_send);
//            ExtractData(&unitree_Data.unitree_recv, motor_rx_temp);
//        }
////        ModifyData(&unitree_Data.unitree_send, 0, 0, 0, (unitree_Data.zero_pose + pos), 0.02, 3);
//        UnitreeSend(&unitree_Data.unitree_send);
//        ExtractData(&unitree_Data.unitree_recv, motor_rx_temp);
//    }
//    if (flag == 2) // ����
//    {
//        if ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) < pos)
//            return;
//        while ((unitree_Data.unitree_recv.Pos - unitree_Data.zero_pose) > (pos + 0.08))
//        {
////            ModifyData(&unitree_Data.unitree_send, 0, 0, -0.2, 0, 0, 3);
//            UnitreeSend(&unitree_Data.unitree_send);
//            ExtractData(&unitree_Data.unitree_recv, motor_rx_temp);
//        }
////        ModifyData(&unitree_Data.unitree_send, 0, 0, 0, (unitree_Data.zero_pose + pos), 0.02, 3);
//        UnitreeSend(&unitree_Data.unitree_send);
//        ExtractData(&unitree_Data.unitree_recv, motor_rx_temp);
//    }
//}
