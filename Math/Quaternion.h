#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

extern float	q1, q2, q3, q4;
//四元数归一化
void Q_normalize(void);
// 返回该四元数的等效旋转矩阵中的重力分量
void vector_gravity(Vector3f *v);
//一阶龙格库塔法更新四元数
void Runge_Kutta_1st(Vector3f *g, float deltaT);
//四元数转欧拉角
void to_euler(float *roll, float *pitch, float *yaw);
#endif 














