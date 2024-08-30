//#include "main.h"
//#include "motor_msg.h"
//#include <string.h>
//#include <stdio.h>
//#include "usart.h"
//#include "unitreeA1_cmd.h"

//#define PI 3.141592654f

//extern UART_HandleTypeDef huart6;
//extern DMA_HandleTypeDef hdma_usart6_rx;
////uint8_t Unitree_rx6_buf[2][Unitree_RX_BUF_NUM];//双缓冲数组

//motor_send_t cmd_motor;        // 电机发送数据体
//motor_recv_t Date_motor;       // 电机接收数据体

//// CRC校验位的代码
///*
//可见crc32_core函数的第一个参数是uint32_t型的指针 ptr，uint32_t型即是 4 字
//节长度的无符号整形数，ptr即表示需要进行 CRC 校验的数据指针。而另一个参数len则
//表示需要进行 CRC 校验的数据长度，由于我们发送的命令去掉最后的 CRC 校验位之后
//还有 30 个字节，也就是包含 7 个完整的uint32_t型，所以在计算发送命令的 CRC 时需
//要令len=7。
//*/
//uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
//{
//    uint32_t bits;
//    uint32_t i;
//    uint32_t xbit = 0;
//    uint32_t data = 0;
//    uint32_t CRC32 = 0xFFFFFFFF;
//    const uint32_t dwPolynomial = 0x04c11db7;
//    for (i = 0; i < len; i++)
//    {
//        xbit = 1 << 31;
//        data = ptr[i];
//        for (bits = 0; bits < 32; bits++)
//        {
//            if (CRC32 & 0x80000000)
//            {
//                CRC32 <<= 1;
//                CRC32 ^= dwPolynomial;
//            }
//            else
//                CRC32 <<= 1;
//            if (data & xbit)
//                CRC32 ^= dwPolynomial;

//            xbit >>= 1;
//        }
//    }
//    return CRC32;
//}

//// 电机位置模式
//void modfiy_position_cmd(motor_send_t *send, uint8_t id, float Pos, float KP, float KW)
//{
//    send->hex_len = 34;

//    send->mode = 10;
//	send->id   = id;

//    send->Pos  = 2.0f * PI / 360.0f * 9.1f * Pos;  //此时pos单位为角度制；弧度制为 9.1 * Pos
//    send->W    = 0;
//    send->T    = 0.0;
//    send->K_P  = KP;
//    send->K_W  = KW;
//}

//// 电机速度修改
//void modfiy_speed_cmd(motor_send_t *send, uint8_t id, float Omega)
//{

//    send->hex_len = 34;

//    send->mode = 10;
//	send->id   = id;

//    send->Pos  = 0;
//    send->W    = Omega;
//    send->T    = 0.0;
//    send->K_P  = 0.0;
//    send->K_W  = 3.0;
//}

//// 电机力矩模式
//void modfiy_torque_cmd(motor_send_t *send, uint8_t id, float torque)
//{

//    send->hex_len = 34;

//    send->mode = 10;
//	send->id   = id;

//    send->Pos  = 0.0;
//    send->W    = 0.0;
//    send->T    = torque;
//    send->K_P  = 0.0;
//    send->K_W  = 0.0;
//}
//uint8_t Date[78];       // 接收数据

//// 电机发送接收函数
//void unitreeA1_rxtx(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart6)
//    {
//        uint8_t Cmd[34]; 		// 发送数据
////        uint8_t Date[78];       // 接收数据

//        cmd_motor.motor_send_data.head.start[0] = 0xFE;
//        cmd_motor.motor_send_data.head.start[1] = 0xEE;
//        cmd_motor.motor_send_data.head.motorID  = cmd_motor.id;
//        cmd_motor.motor_send_data.head.reserved = 0x00;

//        cmd_motor.motor_send_data.Mdata.mode      = cmd_motor.mode;  // mode = 10
//        cmd_motor.motor_send_data.Mdata.ModifyBit = 0xFF;
//        cmd_motor.motor_send_data.Mdata.ReadBit   = 0x00;
//        cmd_motor.motor_send_data.Mdata.reserved  = 0x00;
//        cmd_motor.motor_send_data.Mdata.Modify.F  = 0;
//        cmd_motor.motor_send_data.Mdata.T         = cmd_motor.T * 256;
//        cmd_motor.motor_send_data.Mdata.W         = cmd_motor.W * 128;
//        cmd_motor.motor_send_data.Mdata.Pos       = (int)((cmd_motor.Pos / 2 / PI) * 16384.0f); // 单位 rad
//        cmd_motor.motor_send_data.Mdata.K_P       = cmd_motor.K_P * 2048;
//        cmd_motor.motor_send_data.Mdata.K_W       = cmd_motor.K_W * 1024;

//        cmd_motor.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
//        cmd_motor.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
//        cmd_motor.motor_send_data.Mdata.Res[0] = cmd_motor.Res;

//        cmd_motor.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&cmd_motor.motor_send_data), 7); // CRC校验

//        memcpy(Cmd, &cmd_motor.motor_send_data, 34);
//        
//        // HAL库 DMA 发送数据 + 接收数据
//		HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_SET);
//		
//        HAL_UART_Transmit_DMA(&huart6, Cmd, 34);
//        HAL_Delay(10);
//		HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_SET);
//        HAL_UART_Receive_DMA(&huart6, Date, 78);

//        // 接受数据处理
//        Date_motor.motor_recv_data.head.motorID = Date[2];  
//        Date_motor.motor_recv_data.Mdata.mode   = Date[4];  
//        Date_motor.motor_recv_data.Mdata.Temp   = Date[6];
//        Date_motor.motor_recv_data.Mdata.MError = Date[7]; 
//        Date_motor.motor_recv_data.Mdata.T      = Date[13] << 8  | Date[12]; // 反拼
//        Date_motor.motor_recv_data.Mdata.W      = Date[15] << 8  | Date[14]; // 反拼
//        Date_motor.motor_recv_data.Mdata.Acc    = Date[27] << 8  | Date[26]; // 反拼
//        Date_motor.motor_recv_data.Mdata.Pos    = Date[33] << 24 | Date[32] << 16 | Date[31] << 8 | Date[30];  // 反拼

//        Date_motor.motor_id = Date_motor.motor_recv_data.head.motorID;                               // ID     正确
//        Date_motor.mode     = Date_motor.motor_recv_data.Mdata.mode;                                 // mode   正确
//        Date_motor.Temp     = Date_motor.motor_recv_data.Mdata.Temp;                                 // Temp   正确 (整数)
//        Date_motor.MError   = Date_motor.motor_recv_data.Mdata.MError;                               // MError 正确
//        Date_motor.T        = (float) Date_motor.motor_recv_data.Mdata.T / 256;                      // T      正确
//        Date_motor.Pos      = (float) (Date_motor.motor_recv_data.Mdata.Pos / (16384.0f/2/PI));      // Pos    正确
//        Date_motor.W        = (float) Date_motor.motor_recv_data.Mdata.W / 128;                      // W      正确 (小数)
//        Date_motor.Acc      = (float) Date_motor.motor_recv_data.Mdata.Acc;                          // Acc    貌似正确 (需要VOFA打印测试看是否连续)
//    }
//}

////宇树电机串口初始化，设置双缓冲内存地址
////void Unitree_Usart6_Init()
////{
////	//使能DMA串口接收
////	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
////	//使能空闲中断
////	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
////	//失效DMA
////	__HAL_DMA_DISABLE(&hdma_usart6_rx);
////	
////	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
////	{
////		__HAL_DMA_DISABLE(&hdma_usart6_rx);
////	}
////	
////	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
////	//内存缓冲区1
////	hdma_usart6_rx.Instance->M0AR = (uint32_t)(Unitree_rx6_buf[0]);
////	//内存缓冲区2
////	hdma_usart6_rx.Instance->M1AR = (uint32_t)(Unitree_rx6_buf[1]);
////	//数据长度
////	hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
////	//使能双缓冲区
////	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
////	//使能DMA
////	__HAL_DMA_ENABLE(&hdma_usart6_rx);
////}

////双缓冲设置（乒乓缓冲），在空闲的时候转换下次接收的缓冲
////void USER_UART6_IDLECallback(UART_HandleTypeDef* huart)
////{
////	if(huart == &huart6)
////	{
////		static uint16_t this_time_rx_len = 0;
////		
////		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
////		{
////			//失效DMA
////			__HAL_DMA_DISABLE(&hdma_usart6_rx);
////			//获取接收数据长度,长度 = 设定长度 - 剩余长度
////			this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
////			//重新设定数据长度
////			hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
////			//设定缓冲区1
////			DMA2_Stream1->CR |= DMA_SxCR_CT;
////			//使能DMA
////			__HAL_DMA_ENABLE(&hdma_usart6_rx);
////			
////			if(this_time_rx_len == Unitree_RX_BUF_NUM)
////			{
////				
////				
////				//data_rx_copy(&motor_data_rx6, Unitree_rx6_buf[0]);
////			}
////		}
////		else
////		{
////			//失效DMA
////			__HAL_DMA_DISABLE(&hdma_usart6_rx);
////			//获取接收数据长度,长度 = 设定长度 - 剩余长度
////			this_time_rx_len = 2 * Unitree_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;
////			//重新设定数据长度
////			hdma_usart6_rx.Instance->NDTR = 2 * Unitree_RX_BUF_NUM;
////			//设定缓冲区0
////			DMA2_Stream1->CR &= ~(DMA_SxCR_CT);//////////////////
////			//使能DMA
////			__HAL_DMA_ENABLE(&hdma_usart6_rx);
////			
////			if(this_time_rx_len == Unitree_RX_BUF_NUM)
////			{
////				
////				
////				//data_rx_copy(&motor_data_rx6, Unitree_rx6_buf[1]);
////			}
////		}
////	}
////}

///***********************************************中断区*********************************************/
////int unitree_cnt = 0;
////void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
////{
////	HAL_GPIO_WritePin(A1Motor_DE_GPIO_Port, A1Motor_DE_Pin, GPIO_PIN_RESET);
////	HAL_GPIO_WritePin(A1Motor_RE_GPIO_Port, A1Motor_RE_Pin, GPIO_PIN_RESET);
////	//ExtractData(&unitree_Data.unitree_data_rx,&motor_data_rx6);
////	unitree_cnt++;
////}
