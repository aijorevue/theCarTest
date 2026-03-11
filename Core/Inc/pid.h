#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Target;    // 目标值
    float Actual;    // 当前实际值
    float Out;       // PID 输出值
    
    float Kp, Ki, Kd;
    
    float Error0;    // 当前误差
    float Error1;    // 上一次误差
    float ErrorInt;  // 误差积分
    
    float OutMax;    // 输出上限
    float OutMin;    // 输出下限
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);

#endif