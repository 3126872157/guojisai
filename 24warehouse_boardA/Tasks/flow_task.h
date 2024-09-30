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

#endif