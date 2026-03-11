#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h" // 包含 HAL 库定义

// 1. 声明变量（使用 extern 关键字，表示这些变量在别处定义，此处仅引用）
extern uint8_t s_L2, s_L1, s_M, s_R1, s_R2;

// 2. 声明读取函数
uint8_t Sensor_Read_Avoid_L(void);
uint8_t Sensor_Read_Avoid_R(void);
uint8_t Sensor_Read_Tracking(void);

#endif