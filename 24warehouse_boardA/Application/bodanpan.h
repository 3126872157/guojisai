#ifndef __BODANPAN_H
#define __BODANPAN_H

#include "main.h"
#include "CAN_receive.h"
#include "pid.h"

#define BOX_NUM 10
#define ANGLE_PER_BOX 360/BOX_NUM

typedef struct 
{
	int32_t angle_set;         //目标角度(0-360度)
	int8_t position;           //当前位置(0-9)对应10个格子
	uint8_t IC_date[BOX_NUM];  //储球数据
	uint8_t box_state[BOX_NUM];//格子状态（0为空 1为有球）
	uint8_t *IC_data_ptr;      //储球数据指针
	fp32 code_set;             //目标转子机械角度(用作pid计算)
	fp32 code_now;             //当前转子机械角度(用作pid计算)
	fp32 speed_set;            //目标速度设置，用于速度环pid
	fp32 speed;                //当前速度
}Pan_t;


extern Pan_t bodanpan;

void bodanpan_init(void);
void bodanpan_position_set(uint8_t direction,uint8_t num);//向direction方向转动num个格子
void bodanpan_find_ball(uint8_t x,uint8_t y);//根据行列号旋转拨蛋盘到对应球的位置，便于后续取出该球



#endif
