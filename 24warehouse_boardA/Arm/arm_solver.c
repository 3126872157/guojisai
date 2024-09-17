//
// Created by ken on 24-9-16.
//
#include "arm_solver.h"


struct arm_solver *arm;

void arm_solver_init(struct arm_solver *solver, float b0, float b1, float l0, float l1, float l2)
{
    solver->b0 = b0 * D2R;
    solver->b1 = b1 * D2R;
    solver->l0 = l0;
    solver->l1 = l1;
    solver->l2 = l2;
}

unsigned char arm_solver_analyze(struct arm_solver *solver, float a_total, float x, float y)
{
    solver->a_total = a_total * D2R;
    solver->x = x;
    solver->y = y;

    float x0 = 0;
    float x1 = 0;
    float x2 = 0;
    float y0 = 0;
    float y1 = 0;
    float y2 = 0;

    // 解二元一次方程法，会有双解，但只会解出不想要的那一个
    // float tmp = asinf((solver->x * solver->x + solver->y * solver->y - 2 * solver->l2 * (solver->x * sinf(solver->a_total) + solver->y * cos(solver->a_total)) + solver->l2 * solver->l2 + solver->l1 * solver->l1 - solver->l0 * solver->l0) / (2 * solver->l1 * sqrt(solver->l2 * solver->l2 + solver->x * solver->x + solver->y * solver->y - 2 * solver->l2 * (solver->x * sinf(solver->a_total) + solver->y * cos(solver->a_total))))) - atanf((solver->y - solver->l2 * cos(solver->a_total)) / (solver->x - solver->l2 * sinf(solver->a_total)));
    // solver->a2 = solver->a_total - tmp;
    // solver->a0 = asinf((solver->x - solver->l2 * sinf(solver->a_total) - solver->l1 * sinf(tmp)) / solver->l0);
    // solver->a1 = solver->a_total - solver->a0 - solver->a2;

    // 余弦定理解法
    x2 = solver->l2 * sinf(solver->a_total);
    y2 = sqrtf(solver->l2 * solver->l2 - x2 * x2);
    x1 = x - x2;
    y1 = y - y2;
    float l = sqrtf(x1 * x1 + y1 * y1);
    float a = atanf(y1 / x1);
    float tmp = (solver->l0 * solver->l0 + l * l - solver->l1 * solver->l1) / (2 * solver->l0 * l);
    float a_cos = acosf(tmp);
    solver->a0 = PI_2 - a_cos - a;
    x0 = solver->l0 * sinf(solver->a0);
    y0 = solver->l0 * cosf(solver->a0);
    x1 = x1 - x0;
    y1 = y1 - y0;
    solver->a1 = PI_2 - solver->a0 - atanf(y1 / x1);
    solver->a2 = solver->a_total - solver->a0 - solver->a1;

    // 因为机械臂第0个关节弯曲，所以要调整一下
    solver->a0 = (solver->a0 - PI + solver->b0 + solver->b1) * R2D;
    solver->a1 = (solver->a1 - solver->b1) * R2D;
    solver->a2 = solver->a2 * R2D;

    // 角度限位保护
    if(fabs(solver->a1) > A1_MAX_ANGLE)
        return 2;
    if (solver->a0 > A0_MAX_ANGLE || solver->a1 < -80.0)
        return 1;

    return 0;
}
