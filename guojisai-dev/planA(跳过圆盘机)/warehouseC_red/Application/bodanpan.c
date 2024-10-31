#include "bodanpan.h"


Pan_t bodanpan;

void bodanpan_init()
{
	for(uint8_t i = 0;i < BOX_NUM; i++)
	{
		bodanpan.IC_date_pan[i] = 0;
		bodanpan.box_state[i] = 0;
	}
	bodanpan.angle_set = 0;
	bodanpan.position = 0;
	bodanpan.IC_data_ptr = bodanpan.IC_date_pan;

//test
//	bodanpan.date[0] = 0x12;
//	bodanpan.date[1] = 0x23;
//	bodanpan.date[2] = 0x21;
//	bodanpan.date[3] = 0x11;
//	bodanpan.date[4] = 0x32;
//	bodanpan.date[5] = 0x13;
//	bodanpan.date[6] = 0x33;
//	bodanpan.date[7] = 0x22;
//	bodanpan.date[8] = 0x31;	
//  应该是用来调试的数据
	
}

void bodanpan_position_set(int8_t direction, uint8_t num)//向direction方向转动num个格子
{
	if(direction == 1)
	{
		bodanpan.angle_set += num * ANGLE_PER_BOX;
		bodanpan.position += num;
	}
	else
	{
		bodanpan.angle_set -= num * ANGLE_PER_BOX;
		bodanpan.position -= num;
	}
	bodanpan.code_set = bodanpan.angle_set * 819.2;//360度对应8192*36，所以一度对应819.2转子机械角度
	if(bodanpan.position < 0) bodanpan.position += 10;
	if(bodanpan.position >= 10) bodanpan.position %= 10;
}


uint8_t bodanpan_find_ball(uint8_t x,uint8_t y)//根据行(x)列(y)号旋转拨蛋盘到对应球的位置，便于后续取出该球
{                                                
	int8_t Num,Direction = 1,i;
	for(i = 0;i < BOX_NUM;i++)
	{
		if(bodanpan.IC_date_pan[i] == x * 16 + y )
		{
			Num = bodanpan.position-i;
			
			if(Num < -5)
			{
				Direction = -1;
				Num += 10;
			}
			else if(Num > 5)
			{
				Direction = 1;
				Num = 10 - Num;
			}
			else if(Num > 0 && Num <= 5)
			{
				Direction = -1;
			}
			else if(Num < 0 && Num >= -5)
			{
				Direction = 1;
				Num = -Num;
			}
			bodanpan_position_set(Direction, Num);
			bodanpan.IC_date_pan[i] = 0x00;
			return 0;
		}
	}
	if(i == BOX_NUM)
	{
		return 1;
	}
}
