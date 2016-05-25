#ifndef __SKYBOARD_IMU_H
#define __SKYBOARD_IMU_H

#include "Config.h"
//ŷ���Ǳ�ʾ�ķ�������̬
//extern Vector3f angle;

extern Vector3f Gyro, Acc; 
extern Vector3f EarthAcc;
//extern LPF2ndData_t Acc_lpf_2nd;
extern float magHold;

void IMU_Init(void);
//���´���������
void updateSensor(void);
//�����������̬
void getAttitude(void);
#endif

