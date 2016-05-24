#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>

extern float	q1, q2, q3, q4;
//��Ԫ����һ��
void Q_normalize(void);
// ���ظ���Ԫ���ĵ�Ч��ת�����е���������
void vector_gravity(Vector3f *v);
//һ�����������������Ԫ��
void Runge_Kutta_1st(Vector3f *g, float deltaT);
//��Ԫ��תŷ����
void to_euler(float *roll, float *pitch, float *yaw);
#endif 














