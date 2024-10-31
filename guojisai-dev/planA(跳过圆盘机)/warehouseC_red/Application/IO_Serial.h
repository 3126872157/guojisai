#include "main.h"
#include "tim.h"

#define IO_USART_SENDDELAY_TIME  8.680555556    //115200波特率计算值，这里没用到，定时器4375分频，在设置自动重装值为6即可得到115200Hz

#define Serial_TIM htim2
#define DELAY_TIM  htim3	//生成微秒延时器用的

//枚举类型标记当前位
enum{
 COM_START_BIT,	//0
 COM_D0_BIT,	//1
 COM_D1_BIT,
 COM_D2_BIT,
 COM_D3_BIT,
 COM_D4_BIT,
 COM_D5_BIT,
 COM_D6_BIT,
 COM_D7_BIT,	//8
 COM_STOP_BIT,
};

//GPIO TX脚宏定义
#define IO_SERIAL_TX(n)  if(n) HAL_GPIO_WritePin(Serial_RX_GPIO_Port, Serial_RX_Pin, GPIO_PIN_SET); \
						else  HAL_GPIO_WritePin(Serial_RX_GPIO_Port, Serial_RX_Pin, GPIO_PIN_RESET);
//GPIO RX引脚宏定义
#define IO_SERIAL_RX  HAL_GPIO_ReadPin(Serial_RX_GPIO_Port, Serial_RX_Pin)

void my_delay_us(volatile uint32_t nTime);
