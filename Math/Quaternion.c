#include "Math.h"

float	q1=1, q2=0, q3=0, q4=0;

// ���ظ���Ԫ���ĵ�Ч��ת�����е���������
void vector_gravity(Vector3f *v)
{
  v->x = 2*(q2*q4 - q1*q3);								
  v->y = 2*(q1*q2 + q3*q4);						  
  v->z = 1-2*(q2*q2 + q3*q3);
}


//��Ԫ����һ��
void Q_normalize(void)
{
	float length; 
	length= pythagorous4(q1, q2, q3, q4);
	q1 /= length;
	q2 /= length;
	q3 /= length;
	q4 /= length;
}

//һ�����������������Ԫ��
void Runge_Kutta_1st(Vector3f *g, float deltaT)
{
  q1 += 0.5 * (-q2*g->x - q3*g->y - q4*g->z)* deltaT;
  q2 += 0.5 * (q1*g->x + q3*g->z - q4*g->y)* deltaT;
  q3 += 0.5 * (q1*g->y - q2*g->z + q4*g->x)* deltaT;
  q4 += 0.5 * (q1*g->z + q2*g->y - q3*g->x)* deltaT;	
}

//��Ԫ��תŷ����
void to_euler(float *roll, float *pitch, float *yaw)
{
    if (roll) {
        *roll = degrees(atan2f(2.0f*(q1*q2 + q3*q4),1 - 2.0f*(q2*q2 + q3*q3)));
    }
    if (pitch) {
        // ʹ��safe_asin()������pitch�ӽ�90/-90ʱ�����
        *pitch = degrees(safe_asin(2.0f*(q1*q3 - q2*q4)));
    }
    if (yaw) {
        *yaw = degrees(atan2f(2.0f*(q2*q3 - q1*q4), 2.0f*(q1*q1 + q2*q2) - 1));
    }
}









