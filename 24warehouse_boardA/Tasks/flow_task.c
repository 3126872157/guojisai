#include "flow_task.h"

//控制底盘运动
extern chassis_move_t chassis_move;

//控制变量列表，格式如下
//底盘或者机械臂   x方向走的距离   y方向走的距离   到达判断误差值   底盘运动模式
//1为底盘0为机械臂    单位cm         每次清零          单位cm       3是走和转
TargetPoints targ_point[] = {
	{1,		 20,	 0,		5,		3},
	{1,		 0,		20,		5,		3},
	{1,		-20,	 0,		5,		3},
	{1,		 0,		-20,	5,		3}
};

//当前所在位置序号
uint8_t currentTargIndex = 0;
//完成任务标志
uint8_t isFinished = 0;

//void vFlowTask(void* pvParameters)
//{
//	while(1)
//	{
//		//判断所有步骤是否走完
//		if(currentTargIndex < sizeof(targ_point) / sizeof(TargetPoints))
//		{
//			//读出当前目标
//			TargetPoints target = targ_point[currentTargIndex];
//			
//			//目前动底盘
//			if(target.mode == 1)
//			{
//				//设置底盘运动目标
//				chassis_move.chassis_mode = target.chassis_mode;
//				chassis_move.x_set = target.x;
//				chassis_move.y_set = target.y;
//				
//				//判断误差
//				float distance = sqrt(pow(target.x - chassis_move.x, 2) + pow(target.y - chassis_move.y, 2));
//				if(distance < target.tolerance)
//				{
//					currentTargIndex ++;
//				}
//			}
//		}
//		else
//		{
//			isFinished ++;
//			//删除当前任务
//			vTaskDelete(NULL);
//		}
//	}
//}