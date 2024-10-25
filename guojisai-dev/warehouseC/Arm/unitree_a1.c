#include "math.h"
#include "string.h"
#include "arm_ctrl.h"
#include "A1_motor_msg.h"

#define PI 3.1415926f
#define MAX_TORQUE 2.0f

uint8_t unitree_rx_buf[2][Unitree_RX_BUF_NUM]; // dma接收缓冲
ServoComdDataV3 motor_rx_temp;					// 解包缓冲，从dma缓冲中复制出来，再导入接收的结构体中

//与本文件无关，串口1dma发送标志位
extern volatile uint8_t usart_dma_tx_over;

// CRC校验
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

// 校验数据并把接收buf复制到接收数据中
void data_rx_copy(ServoComdDataV3 *motor_data_rx, uint8_t *data)
{
    uint32_t CRC_data = *((uint32_t *)(data + 74));

    if (CRC_data == crc32_core((uint32_t *)data, 18))
    {
        memcpy(motor_data_rx, data, Unitree_RX_BUF_NUM);
    }
}

// 宇树A1电机串口初始化
void unitree_Uart_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能DMA串口接收
    SET_BIT(UNITREE_MOTOR_HUART.Instance->CR3, USART_CR3_DMAR);
    // 使能空闲中断
    __HAL_UART_ENABLE_IT(&UNITREE_MOTOR_HUART, UART_IT_IDLE);
    // 失效DMA
    __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);

    while (UNITREE_MOTOR_HDMA_RX.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
    }

    UNITREE_MOTOR_HDMA_RX.Instance->PAR = (uint32_t) & (UNITREE_MOTOR_UART->DR);
    // 内存缓冲区1
    UNITREE_MOTOR_HDMA_RX.Instance->M0AR = (uint32_t)(rx1_buf);
    // 内存缓冲区2
    UNITREE_MOTOR_HDMA_RX.Instance->M1AR = (uint32_t)(rx2_buf);
    // 数据长度
    UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * dma_buf_num;
    // 使能双缓冲区
    SET_BIT(UNITREE_MOTOR_HDMA_RX.Instance->CR, DMA_SxCR_DBM);
    // 使能DMA
    __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);
}

// 空闲中断执行双缓冲交换（乒乓缓冲），弱定义在it.h
void USER_Unitree_A1_Motor_UART_IDLECallback(UART_HandleTypeDef *huart)
{
    if (huart == &UNITREE_MOTOR_HUART)
    {
        static uint16_t this_time_rx_len = 0;

        if ((UNITREE_MOTOR_HDMA_RX.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // 失能DMA
            __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
            // 获取接收数据长度 长度 = 设定长度 - 剩余长度（NDTR）
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - UNITREE_MOTOR_HDMA_RX.Instance->NDTR;
            // 重新设定剩余数据长度
            UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // 设定缓冲区1
            DMA2_Stream1->CR |= DMA_SxCR_CT;
            // 使能DMA
            __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);
            // 若实际接收长度与理论接受长度一致
            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, unitree_rx_buf[0]);
            }
        }
        else
        {
            // 失能DMA
            __HAL_DMA_DISABLE(&UNITREE_MOTOR_HDMA_RX);
            // 同上
            this_time_rx_len = 2 * Unitree_RX_BUF_NUM - UNITREE_MOTOR_HDMA_RX.Instance->NDTR;
            // 重新设定数据长度
            UNITREE_MOTOR_HDMA_RX.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
            // 设定缓冲区0
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            // 使能DMA
            __HAL_DMA_ENABLE(&UNITREE_MOTOR_HDMA_RX);

            if (this_time_rx_len == Unitree_RX_BUF_NUM)
            {
                data_rx_copy(&motor_rx_temp, unitree_rx_buf[1]);
            }
        }
    }
}

// 发送中断函数，处理485转ttl模块状态，放在it.c文件里吧
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UNITREE_MOTOR_UART)
	{
//		UBaseType_t uxSavedInterruptStatus;
//		uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
//		taskENTER_CRITICAL_FROM_ISR();
		
		// 发送完成进入tx中断回调，拉低RE电平，使能485转ttl模块接收状态
		HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_RESET);
		
//		UNITREE_MOTOR_HUART.gState = HAL_UART_STATE_READY;
//		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
	}
}

// 发送电机指令
void UnitreeSend(motor_send_t *motor)
{
    // 拉高DE电平，使能485转ttl模块的发送模式，失能接受模式；此模块不能同时处于两种模式中
    HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_SET);

	//这改为DMA
	
//	taskENTER_CRITICAL();
	//这里不需要延时，用IT发送会因为INS_TASK抢占而卡住
    HAL_UART_Transmit_DMA(&UNITREE_MOTOR_HUART, (uint8_t *)&(motor->motor_send_data), 34);
//	taskEXIT_CRITICAL();
}

// 解包电机返回数据
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

// 打包电机发送数据
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

// 力矩控制模式，一般用力矩模式加上pid
void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque)
{
	//限幅2N·M
	if(torque >= MAX_TORQUE)
	{
		torque = MAX_TORQUE;
	}
	else if(torque <= -MAX_TORQUE)
	{
		torque = -MAX_TORQUE;
	}
	
    // 用户设置的数据
    send->id = id;
    send->mode = 10;
    send->Pos = 0.0;
    send->W = 0.0;
    send->T = torque;
    send->K_P = 0.0;
    send->K_W = 0.0;

    WriteData(send);
}

// 位置控制模式，这个模式很鸡肋，偏差太大会咔咔，转个180就不行
void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW)
{
    send->id = id;
    send->mode = 10;
    send->Pos = 9.1f * Pos; // 弧度制
    send->W = 0;
    send->T = 0.0;
    send->K_P = KP; // 官方推荐0.02，实测再小一点会好
    send->K_W = KW; // 官方推荐3.0，实测1.0及一下会比较好

    WriteData(send);
}

// 速度控制模式
void modfiy_speed_cmd(motor_send_t *send, uint8_t id, float Omega, float kw)
{

    send->id = id;
    send->mode = 10;
    send->Pos = 0;
    send->W = Omega;
    send->T = 0.0;
    send->K_P = 0.0;
    send->K_W = kw;	// 推荐值3.0

    WriteData(send);
}

// 混合模式：啥都有（公式：Output = K_P * delta_Pos + K_W * delta_W + T）
void modfiy_mix_cmd(motor_send_t *send, uint8_t ID, float Torque, float POS, float W, float KP, float KW)
{
    send->id = ID;
    send->mode = 10; // 默认混控模式
    send->T = Torque;
    send->W = W * 9.1f;
    send->Pos = 9.1f * POS; // 弧度制
    send->K_P = KP;
    send->K_W = KW;
   
	WriteData(send);
}

//it.c里的内容
//void USART6_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART6_IRQn 0 */

//  /* USER CODE END USART6_IRQn 0 */
//  HAL_UART_IRQHandler(&huart6);
//  /* USER CODE BEGIN USART6_IRQn 1 */

//	//此为空闲中断，此处须在.h文件里声明
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
//	if(huart6.Instance->SR & UART_FLAG_RXNE)//接收到数据：状态寄存器里的接受寄存器非空为1（非空了）
//	{
//		__HAL_UART_CLEAR_PEFLAG(&huart6);	//此函数用来清除IDLE标志位
//	}
//	else if(USART6->SR & UART_FLAG_IDLE)	//查询状态寄存器，此时串口空闲
//	{
//		// 清除空闲中断标志（否则会一直重复进入中断）
//		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                    
//		// 调用中断处理函数，此函数在宇树电机文件里
//		USER_UART6_IDLECallback(&huart6); 
//	}
//}

///* USER CODE END 1 */
