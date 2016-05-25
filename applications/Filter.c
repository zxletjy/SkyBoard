//�˲�����غ���
#include "Filter.h"

/*----------------------���׵�ͨ�˲���ϵ������-------------------------*/
void LPF_2nd_Factor_Cal(LPF2ndData_t* lpf_data)
{
	//��ֹƵ��:30Hz ����Ƶ��:500Hz
	lpf_data->b0 = 0.1883633f;
	lpf_data->a1 = 1.023694f;
	lpf_data->a2 = 0.2120577f;
}

/*----------------------���׵�ͨ�˲���------------------------*/
Vector3f LPF_2nd(LPF2ndData_t* lpf_2nd, Vector3f newData)
{
	Vector3f lpf_2nd_data = {0,0,0};
	
	lpf_2nd_data.x = newData.x * lpf_2nd->b0 + lpf_2nd->lastout.x * lpf_2nd->a1 - lpf_2nd->preout.x * lpf_2nd->a2;
	lpf_2nd_data.y = newData.y * lpf_2nd->b0 + lpf_2nd->lastout.y * lpf_2nd->a1 - lpf_2nd->preout.y * lpf_2nd->a2;
	lpf_2nd_data.z = newData.z * lpf_2nd->b0 + lpf_2nd->lastout.z * lpf_2nd->a1 - lpf_2nd->preout.z * lpf_2nd->a2;
	
	lpf_2nd->preout = lpf_2nd->lastout;
	lpf_2nd->lastout = lpf_2nd_data;
	
	return lpf_2nd_data;
}

/*----------------------�����˲���ϵ������-------------------------*/
float CF_Factor_Cal(float deltaT, float tau)
{
	return tau / (deltaT + tau);
}

