//
// Created by ken on 24-9-16.
//
#include "arm_solver.h"
#include "main.h"
#include "struct_typedef.h"

bool_t nan_error = 0;

void arm_solver_init(struct arm_solver *solver, float b0, float b1, float l0, float l1, float l2)
{
    solver->b0 = b0 * D2R;
    solver->b1 = b1 * D2R;
    solver->l0 = l0;
    solver->l1 = l1;
    solver->l2 = l2;
}

//输入参数：解算结构体、总角度(角度制)、x、y坐标值(单位：毫米)
uint8_t arm_solver_analyze(struct arm_solver *solver, float a_total, float x, float y)
{
	if(nan_error != 1)
	{
		if(a_total < 30 || a_total >181)
		return 3;
	
		solver->a_total = a_total * D2R;
		solver->x = x;
		solver->y = y;
		
		//角度解算过程量
		float x0 = 0,x1 = 0,x2 = 0,y0 = 0,y1 = 0,y2 = 0;
		
		//解算结果
		float a0;
		float a1;
		float a2;
	
		// 解二元一次方程法，会有双解，但只会解出不想要的那一个
		// float tmp = asinf((solver->x * solver->x + solver->y * solver->y - 2 * solver->l2 * (solver->x * sinf(solver->a_total) + solver->y * cos(solver->a_total)) + solver->l2 * solver->l2 + solver->l1 * solver->l1 - solver->l0 * solver->l0) / (2 * solver->l1 * sqrt(solver->l2 * solver->l2 + solver->x * solver->x + solver->y * solver->y - 2 * solver->l2 * (solver->x * sinf(solver->a_total) + solver->y * cos(solver->a_total))))) - atanf((solver->y - solver->l2 * cos(solver->a_total)) / (solver->x - solver->l2 * sinf(solver->a_total)));
		// solver->a2 = solver->a_total - tmp;
		// solver->a0 = asinf((solver->x - solver->l2 * sinf(solver->a_total) - solver->l1 * sinf(tmp)) / solver->l0);
		// solver->a1 = solver->a_total - solver->a0 - solver->a2;
	
		// 余弦定理解法
		x2 = solver->l2 * sinf(solver->a_total);
		y2 = solver->l2 * cosf(solver->a_total);
		x1 = x - x2;
		y1 = y - y2;
		float l = sqrtf(x1 * x1 + y1 * y1);
		float a = atanf(y1 / x1);
		float tmp = (solver->l0 * solver->l0 + l * l - solver->l1 * solver->l1) / (2 * solver->l0 * l);
		float a_cos = acosf(tmp);
		a0 = PI_2 - a_cos - a;
		x0 = solver->l0 * sinf(a0);
		y0 = solver->l0 * cosf(a0);
		x1 = x1 - x0;
		y1 = y1 - y0;
		//a1 = PI_2 - a0 - atanf(y1 / x1);
		a1 = PI - acosf((solver->l1 * solver->l1 + solver->l0 * solver->l0 - l * l) / (2 * solver->l1 * solver->l0));
		a2 = solver->a_total - a0 - a1;
		
		
	
		// 因为机械臂第0个关节弯曲，所以要调整一下
		a0 = (a0 - PI + solver->b0 + solver->b1);
		a1 = (a1 - solver->b1) * R2D;
		a2 = a2 * R2D;
	
		// 角度限位保护
		if (a0 > 45.0f * D2R || a0 < -80.0 * D2R)
			return 1;
		if(fabs(a1) > A1_MAX_ANGLE)
			return 2;
		
		if(isnan(a0) || isnan(a1) || isnan(a2))
		{
			nan_error = 1;
			return 9;	//nan了
		}
		else//如果不nan了，回到正常状态
			nan_error = 0;
		
		solver->a0 = a0;
		solver->a1 = a1;
		solver->a2 = a2;
	}

    return 0;
}
