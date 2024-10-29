#include "Gray_sensor.h"

uint8_t gray_data[2];

void gray_sensor_read(void)
{
	gray_data[1] = HAL_GPIO_ReadPin(gray_D1_GPIO_Port,gray_D1_Pin);
	gray_data[0] = HAL_GPIO_ReadPin(gray_D2_GPIO_Port,gray_D2_Pin);
}

