#ifndef __SKYBOARD_FILTER_H
#define __SKYBOARD_FILTER_H

#include "Math.h"
typedef struct
{
float b0;
float a1;
float a2;
Vector3f preout;
Vector3f lastout;
}LPF2ndData_t;
//���׵�ͨ�˲���ϵ������
void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data);

//�����˲���ϵ������
float CF_Factor_Cal(float deltaT, float tau);

//���׵�ͨ�˲���
Vector3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData);
#endif

