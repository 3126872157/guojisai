#ifndef FLOW_TASK_H
#define FLOW_TASK_H

#include "main.h"
#include "Chassis_Task.h"
#include "struct_typedef.h"
#include "Gray_sensor.h"
#include "UART_receive.h"
#include "math.h"
#include "freertos.h"

typedef struct {
	uint8_t mode;
	float para1;
	float para2;
	float para3;
	float chassis_V_max;
	uint8_t chassis_mode;
} TargetPoints;


void flow_task(void const * argument);

#endif
