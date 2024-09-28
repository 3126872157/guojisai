#ifndef __BODANPAN_H
#define __BODANPAN_H

#include "main.h"
#include "CAN_receive.h"
#include "pid.h"

#define BOX_NUM 10
#define ANGLE_PER_BOX 360/BOX_NUM

typedef struct 
{
	int32_t angle_set;         //Ŀ��Ƕ�(0-360��)
	int8_t position;           //��ǰλ��(0-9)��Ӧ10������
	uint8_t IC_date[BOX_NUM];  //��������
	uint8_t box_state[BOX_NUM];//����״̬��0Ϊ�� 1Ϊ����
	uint8_t *IC_data_ptr;      //��������ָ��
	fp32 code_set;             //Ŀ��ת�ӻ�е�Ƕ�(����pid����)
	fp32 code_now;             //��ǰת�ӻ�е�Ƕ�(����pid����)
	fp32 speed_set;            //Ŀ���ٶ����ã������ٶȻ�pid
	fp32 speed;                //��ǰ�ٶ�
}Pan_t;


extern Pan_t bodanpan;

void bodanpan_init(void);
void bodanpan_position_set(uint8_t direction,uint8_t num);//��direction����ת��num������
void bodanpan_find_ball(uint8_t x,uint8_t y);//�������к���ת�����̵���Ӧ���λ�ã����ں���ȡ������



#endif
