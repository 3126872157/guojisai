#include "math.h"
#include "string.h"
#include "arm_ctrl.h"
#include "A1_motor_msg.h"

#define PI 3.1415926f
#define MAX_TORQUE 2.0f

uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM]; // dma���ջ���
ServoComdDataV3 motor_rx_temp;					// ������壬��dma�����и��Ƴ������ٵ�����յĽṹ����

//�뱾�ļ��޹أ�����1dma���ͱ�־λ
extern volatile uint8_t usart_dma_tx_over;

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
void unitree_Uart_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // ʹ��DMA���ڽ���
    SET_BIT(UNITREE_MOTOR_HUART.Instance->CR3, USART_CR3_DMAR);
    // ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&UNITREE_MOTOR_HUART, UART_IT_IDLE);
    // ʧЧDMA
    __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);

    while (UNITREE_MOTOR_HDMA_RX.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
    }

    UNITREE_MOTOR_HDMA_RX.Instance->PAR = (uint32_t) & (UNITREE_MOTOR_UART->DR);
    // �ڴ滺����1
    UNITREE_MOTOR_HDMA_RX.Instance->M0AR = (uint32_t)(rx1_buf);
    // �ڴ滺����2
    UNITREE_MOTOR_HDMA_RX.Instance->M1AR = (uint32_t)(rx2_buf);
    // ���ݳ���
    UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * dma_buf_num;
    // ʹ��˫������
    SET_BIT(UNITREE_MOTOR_HDMA_RX.Instance->CR, DMA_SxCR_DBM);
    // ʹ��DMA
    __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);
}

// �����ж�ִ��˫���彻����ƹ�һ��壩����������it.h
void USER_Unitree_A1_Motor_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart == &UNITREE_MOTOR_HUART)
    {
        static uint16_t this_time_rx_len = 0;

        if ((UNITREE_MOTOR_HDMA_RX.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // ʧ��DMA
            __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
            // ��ȡ�������ݳ��� ���� = �趨���� - ʣ�೤�ȣ�NDTR��
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - UNITREE_MOTOR_HDMA_RX.Instance->NDTR;
            // �����趨ʣ�����ݳ���
            UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // �趨������1
            DMA2_Stream1->CR |= DMA_SxCR_CT;
            // ʹ��DMA
            __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);
            // ��ʵ�ʽ��ճ��������۽��ܳ���һ��
            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, unitree_rx_buf[0]);
            }
        }
        else
        {
            // ʧ��DMA
            __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
            // ͬ��
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - UNITREE_MOTOR_HDMA_RX.Instance->NDTR;
            // �����趨���ݳ���
            UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // �趨������0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            // ʹ��DMA
            __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);

            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, unitree_rx_buf[1]);
            }
        }
    }
}

// �����жϺ���������485תttlģ��״̬������it.c�ļ����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UNITREE_MOTOR_UART)
	{
//		UBaseType_t uxSavedInterruptStatus;
//		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
//		taskENTER_CRITICAL_FROM_ISR();
		
		// ������ɽ���tx�жϻص�������RE��ƽ��ʹ��485תttlģ�����״̬
		HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_RESET);
		
//		UNITREE_MOTOR_HUART.gState = HAL_UART_STATE_READY;
//		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	}
}

// ���͵��ָ��
void UnitreeSend(motor_send_t *motor)
{
    // ����DE��ƽ��ʹ��485תttlģ��ķ���ģʽ��ʧ�ܽ���ģʽ����ģ�鲻��ͬʱ��������ģʽ��
    HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_SET);

	//���ΪDMA
	
//	taskENTER_CRITICAL();
	//���ﲻ��Ҫ��ʱ����IT���ͻ���ΪINS_TASK��ռ����ס
    HAL_UART_Transmit_DMA(&UNITREE_MOTOR_HUART, (uint8_t *)&(motor->motor_send_data), 34);
//	taskEXIT_CRITICAL();
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
void modfiy_speed_cmd(motor_send_t *send, uint8_t id, float Omega, float kw)
{

    send->id = id;
    send->mode = 10;
    send->Pos = 0;
    send->W = Omega;
    send->T = 0.0;
    send->K_P = 0.0;
    send->K_W = kw;	// �Ƽ�ֵ3.0

    WriteData(send);
}

// ���ģʽ��ɶ���У���ʽ��Output = K_P * delta_Pos + K_W * delta_W + T��
void modfiy_mix_cmd(motor_send_t *send, uint8_t ID, float Torque, float POS, float W, float KP, float KW)
{
    send->id = ID;
    send->mode = 10; // Ĭ�ϻ��ģʽ
    send->T = Torque;
    send->W = W * 9.1f;
    send->Pos = 9.1f * POS; // ������
    send->K_P = KP;
    send->K_W = KW;
   
	WriteData(send);
}

//it.c�������
//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART6_IRQn 0 */

//  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART6_IRQn 1 */

//	//��Ϊ�����жϣ��˴�����.h�ļ�������
//	USER_USART6_IRQHandler();
//	
//  /* USER CODE END USART6_IRQn 1 */
//}

///* USER CODE BEGIN 1 */

//__weak void USER_UART6_IDLECallback(UART_HandleTypeDef* huart)
//{
//	return;
//}

//void USER_USART6_IRQHandler(void)
//{
//	if(huart6.Instance->SR & UART_FLAG_RXNE)//���յ����ݣ�״̬�Ĵ�����Ľ��ܼĴ����ǿ�Ϊ1���ǿ��ˣ�
//	{
//		__HAL_UART_CLEAR_PEFLAG(&huart6);	//�˺����������IDLE��־λ
//	}
//	else if(USART6->SR & UART_FLAG_IDLE)	//��ѯ״̬�Ĵ�������ʱ���ڿ���
//	{
//		// ��������жϱ�־�������һֱ�ظ������жϣ�
//		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                    
//		// �����жϴ��������˺�������������ļ���
//		USER_UART6_IDLECallback(&huart6); 
//	}
//}

///* USER CODE END 1 */
