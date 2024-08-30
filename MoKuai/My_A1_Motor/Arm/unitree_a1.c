#include "math.h"
#include "string.h"
#include "Arm_Struct.h"

extern unitree_Data_1 unitree_Data;
uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];//?????????
ServoComdDataV3 motor_data_rx6;//???????

void unitree_move(uint8_t flag,float pos,float w)
{
	if(flag == 1)//????
	{
		if((unitree_Data.unitree_data_rx.Pos - unitree_Data.pos0) > pos) return ;
		while((unitree_Data.unitree_data_rx.Pos - unitree_Data.pos0)<(pos - 0.08))
		{
			ModifyData(&unitree_Data.unitree_send,0,10,0,1.2,0,0,3);
			UnitreeSend(&unitree_Data.unitree_send);
			ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
		}
		ModifyData(&unitree_Data.unitree_send,0,10,0,0,(unitree_Data.pos0 + pos),0.02,3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
	}
	if(flag == 2)//????
	{
		if((unitree_Data.unitree_data_rx.Pos - unitree_Data.pos0) < pos) return ;
		while((unitree_Data.unitree_data_rx.Pos - unitree_Data.pos0)>(pos + 0.08))
		{
			ModifyData(&unitree_Data.unitree_send,0,10,0,-0.2,0,0,3);
			UnitreeSend(&unitree_Data.unitree_send);
			ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
		}
		ModifyData(&unitree_Data.unitree_send,0,10,0,0,(unitree_Data.pos0 + pos),0.02,3);
		UnitreeSend(&unitree_Data.unitree_send);
		ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
	}
}
/**
  * @brief  ?????????
  * @param  ????????????
  */
void ModifyData(MOTOR_send* motor_s, uint8_t ID, uint8_t MODE, float Torque, float W, float POS, float KP, float KW)
{
    motor_s->id=ID;
	motor_s->mode=MODE;
	motor_s->T = Torque;
	motor_s->W = W * 9.1f; //?????
	motor_s->Pos = 2.0f * 3.1415926f / 360.0f * 9.1f * POS;  //???pos??¦Ë?????????????? 9.1 * Pos
	motor_s->K_P = KP;
    motor_s->K_W = KW;
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x00;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x00;
    motor_s->motor_send_data.Mdata.reserved = 0x00;
    motor_s->motor_send_data.Mdata.Modify = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T * 256;
    motor_s->motor_send_data.Mdata.W = motor_s->W * 128;
    motor_s->motor_send_data.Mdata.Pos = (int)((motor_s->Pos / 6.2832f) * 16384.0f);
    motor_s->motor_send_data.Mdata.K_P = (int16_t)(motor_s->K_P * 2048);

    motor_s->motor_send_data.Mdata.K_W = (int16_t)(motor_s->K_W * 1024);

    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.CRCdata = crc32_core((uint32_t*)(&(motor_s->motor_send_data)), 7);
}


/**
  * @brief  ??????????????
  * @param  ?????????????
  */
void UnitreeSend(MOTOR_send* motor)
{
	HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	while(HAL_UART_Transmit_IT(&huart6, (uint8_t*) & (motor->motor_send_data), 34) != HAL_OK);
	HAL_Delay(1);
}


/**
  * @brief  ??????????????
  * @param  ??????????????????????
  */
void ExtractData(MOTOR_recv* motor_r, ServoComdDataV3* data_rx)
{
    if(data_rx->head.motorID == 0)motor_r = motor_r + 0;

    if(data_rx->head.motorID == 1)motor_r = motor_r + 1;

    if(data_rx->head.motorID == 2)motor_r = motor_r + 2;

    motor_r->motor_id = data_rx->head.motorID;
    motor_r->mode = data_rx->Mdata.mode;
    motor_r->Temp = data_rx->Mdata.Temp;
    motor_r->MError = data_rx->Mdata.MError;
    motor_r->T = ((float)data_rx->Mdata.T) / 256 * 9.1f;
    motor_r->W = ((float)data_rx->Mdata.W) / 128 / 9.1f;
    motor_r->LW = data_rx->Mdata.LW / 9.1f;
//		motor_r->LW = (float)(data_rx->Mdata.LW>>31);

    motor_r->Acc = ((float)data_rx->Mdata.Acc) / 128 / 9.1f;
    motor_r->Pos = 6.2832f * ((float)data_rx->Mdata.Pos) / 16384 / 9.1f;

    motor_r->gyro[0] = ((float)data_rx->Mdata.gyro[0]) * 0.00107993176f;
    motor_r->gyro[1] = ((float)data_rx->Mdata.gyro[1]) * 0.00107993176f;
    motor_r->gyro[2] = ((float)data_rx->Mdata.gyro[2]) * 0.00107993176f;

    motor_r->acc[0] = ((float)data_rx->Mdata.acc[0]) * 0.0023911132f;
    motor_r->acc[1] = ((float)data_rx->Mdata.acc[1]) * 0.0023911132f;
    motor_r->acc[2] = ((float)data_rx->Mdata.acc[2]) * 0.0023911132f;
}


/**
  * @brief  §µ????????????????????????
  * @param  ??????????????
  */
void data_rx_copy(ServoComdDataV3*motor_data_rx, uint8_t* data)
{
    uint32_t CRC_data = *((uint32_t*)(data + 74));

    if(CRC_data == crc32_core((uint32_t*)data, 18))
    {
        memcpy(motor_data_rx, data, Unitree_RX_BUF_NUM);
    }
}


/**
  * @brief  CRC§µ??
  */
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
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


/**
  * @brief  ???????????
  */
void Unitree_Usart6_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //???DMA???????
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //???????§Ø?
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //?§¹DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);

    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //??H????1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //??H????2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //???????
    hdma_usart6_rx.Instance->NDTR = 2 * dma_buf_num;
    //??????????
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //???DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}

//??????????????????????‰Î????????§Ø??????????¦Í???????
void USER_UART6_IDLECallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart6)
	{
		static uint16_t this_time_rx_len = 0;
		
		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			//?§¹DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);
			//??????????????,???? = ?Ú…???? - ?????
			this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
			//?????Ú…???????
			hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
			//?Ú…??????1
			DMA2_Stream1->CR |= DMA_SxCR_CT;
			//???DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);
			
			if(this_time_rx_len == Unitree_RX_BUF_NUM)
			{
				data_rx_copy(&motor_data_rx6, Unitree_rx6_buf[0]);
			}
		}
		else
		{
			//?§¹DMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);
			//??????????????,???? = ?Ú…???? - ?????
			this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
			//?????Ú…???????
			hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
			//?Ú…??????0
			DMA2_Stream1->CR &= ~(DMA_SxCR_CT);//////////////////
			//???DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);
			
			if(this_time_rx_len == Unitree_RX_BUF_NUM)
			{
				data_rx_copy(&motor_data_rx6, Unitree_rx6_buf[1]);
			}
		}
	}
}

/***********************************************?§Ø???*********************************************/
int unitree_cnt = 0;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_RESET);
	//ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
	unitree_cnt++;
}
