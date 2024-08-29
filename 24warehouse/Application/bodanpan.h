#ifndef __BODANPAN_H
#define __BODANPAN_H


#include "main.h"
#include "CAN_receive.h"
#include "pid.h"


typedef struct 
{
	int32_t angle;//ʵʱ�Ƕ�
	int8_t position;//��ǰλ��
	uint8_t date[9];//��������
	uint8_t box_state[9];//����״̬��0Ϊ�� 1Ϊ����
	uint8_t *ptr;//��������ָ��
	uint32_t angle_desire;
	uint64_t angle_current;
	int32_t v_set;
	int32_t v;
}Pan_t;


extern Pan_t bodanpan;

void pan_init(void);
void turn_box(uint8_t direction,uint8_t num);//ͨ����Ҫת���ĸ��� �����Ӧ�Ƕ�
void put_a_ball(uint8_t x,uint8_t y);//��������³�һ��ָ������ xΪ�� yΪ��



#endif
