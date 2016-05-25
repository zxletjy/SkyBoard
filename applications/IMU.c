//��������̬����
#include "IMU.h"
#include <math.h>

#define GRAVITY         9.80665f    // m/s^2

Vector3f Gyro, Acc;
Vector3f EarthAcc;

float magHold;

//IMU��ʼ��
void IMU_Init(void)
{
	IMU_QCF_Init(&IMU_QCF);
}

//���´���������
void updateSensor(void)
{
	//��ȡ���ٶ�
	Read_Acc_Data();
	//��ȡ���ٶ�
	Read_Gyro_Data();	
	//��ȡ���ٶȣ���λΪ��ÿ��
	//Gyro = mpu6050.Get_Gyro_in_dps();
	Gyro = Gyro_dps;
	//��ȡ���ٶȲ���ֵ
	//Acc = mpu6050.Get_Acc();
	Acc = Acc_ADC;
}
void GetEarthAcc(Vector3f* earth);
//�����������̬
void getAttitude(void)
{
	IMU_QCF_Update(&IMU_QCF, Gyro.x, Gyro.y, Gyro.z, Acc.x, Acc.y, Acc.z, GetSysTime_us());
	GetEarthAcc(&EarthAcc);
}

void GetEarthAcc(Vector3f* earth)
{
	float w, x, y, z;
	float v[3];
	float acc[3];
	
	v[0] = IMU_QCF.acc_lpf_x;//�Ѽ��ٶȼ�����
  v[1] = IMU_QCF.acc_lpf_y;
  v[2] = IMU_QCF.acc_lpf_z;
	
	w = IMU_QCF.Q1;//��Ԫ��
	x = IMU_QCF.Q2;
	y = IMU_QCF.Q3;
	z = IMU_QCF.Q4;
	
	acc[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
	acc[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
	acc[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
	acc[2] += GRAVITY;
	
	earth->x = acc[0];
	earth->y = acc[1];
	earth->z = acc[2];
}

