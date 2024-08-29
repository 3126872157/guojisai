#ifndef __BODANPAN_H
#define __BODANPAN_H


#include "main.h"
#include "CAN_receive.h"
#include "pid.h"


typedef struct 
{
	int32_t angle;//实时角度
	int8_t position;//当前位置
	uint8_t date[9];//储球数据
	uint8_t box_state[9];//格子状态（0为空 1为有球）
	uint8_t *ptr;//储球数据指针
	uint32_t angle_desire;
	uint64_t angle_current;
	int32_t v_set;
	int32_t v;
}Pan_t;


extern Pan_t bodanpan;

void pan_init(void);
void turn_box(uint8_t direction,uint8_t num);//通过需要转动的格数 计算对应角度
void put_a_ball(uint8_t x,uint8_t y);//储球机构吐出一个指定的球 x为行 y为列



#endif
