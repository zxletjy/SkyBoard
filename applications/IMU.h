#ifndef __SKYBOARD_IMU_H
#define __SKYBOARD_IMU_H

#include "Config.h"
//欧拉角表示的飞行器姿态
//extern Vector3f angle;

extern Vector3f Gyro, Acc; 
extern Vector3f EarthAcc;
//extern LPF2ndData_t Acc_lpf_2nd;
extern float magHold;

void IMU_Init(void);
//更新传感器数据
void updateSensor(void);
//计算飞行器姿态
void getAttitude(void);
#endif

