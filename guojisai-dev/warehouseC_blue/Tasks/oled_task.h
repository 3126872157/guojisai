#ifndef __OLED_TASK_H
#define __OLED_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
//#include "main.h"

//任务开始空闲一段时间
#define OLED_TASK_INIT_TIME 357


void oled_task(void const * argument);

#endif
