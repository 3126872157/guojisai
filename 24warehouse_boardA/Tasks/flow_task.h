#ifndef FLOW_TASK_H
#define FLOW_TASK_H

#include "main.h"
#include "Chassis_Task.h"
#include "struct_typedef.h"
#include "math.h"
#include "freertos.h"

typedef struct {
	uint8_t mode;
	float x;
	float y;
	float tolerance;
	uint8_t chassis_mode;
} TargetPoints;

typedef struct
{
	float ball_x;
	float ball_y;
	float ball_distance;
	float QR_x;
	float QR_y;
	float QR_code;
}shijue_Data;

void flow_task(void const * argument);

#endif
