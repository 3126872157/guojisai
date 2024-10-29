//
// Created by ken on 24-9-16.
//
#include <main.h>

#include <math.h>

#ifndef ARM_SOLVER_H
#define ARM_SOLVER_H

#define PI 3.14159265358979323846f
#define PI_2 PI/2.0f
#define R2D 57.295779513082f
#define D2R 0.0174532925f

#define A0_MAX_ANGLE 45.0f
#define A1_MAX_ANGLE 120.0f

struct arm_solver
{
    // 内参
    // 机械臂第0个关节弯曲的角度，钝角
    float b0;
    // 机械臂第0个关节弯曲造成的偏差角度
    float b1;
    // 机械臂连杆长度
    float l0;
    float l1;
    float l2;

    // 输入
    // 末端与垂直方向的夹角
    float a_total;
    // 坐标
    float x;
    float y;

    // 求解
    // 机械臂连杆偏差角度
    float a0;
    float a1;
    float a2;
} ;

void arm_solver_init(struct arm_solver *solver, float b0, float b1, float l0, float l1, float l2);
uint8_t arm_solver_analyze(struct arm_solver *solver, float a_total, float x, float y);

#endif //ARM_SOLVER_H
