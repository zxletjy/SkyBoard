#ifndef _IMU_QCF_H_
#define _IMU_QCF_H_
#include <math.h>
#include "stm32f10x.h"
#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f
typedef struct
{
	float b0;
	float a1;
	float a2;
	float preout_x,preout_y,preout_z;
	float lastout_x,lastout_y,lastout_z;
}LPF2nd_Struct;

typedef struct
{
	float pitch;//��λ�Ƕ�
	float roll;
	float yaw;
	
	float pitch_r;//��λ�ǻ���
	float roll_r;
	float yaw_r;
	
	LPF2nd_Struct Acc_LPF2nd;
	
	float Q1;	//��Ԫ��
	float Q2;
	float Q3;
	float Q4;
	
	float ki;//���ٶ�Ȩ�أ�Խ��������ٶȲ���ֵ����Խ�
	float kp;//����������
	
	float acc_lpf_x;	//���׵�ͨ�˲�֮��ļ��ٶ�����
	float acc_lpf_y;
	float acc_lpf_z;
	
	float gyro_cf_x;	//����֮������
	float gyro_cf_y;
	float gyro_cf_z;
	
}IMU_QCF_Struct;

extern IMU_QCF_Struct IMU_QCF;

extern void IMU_QCF_Init(IMU_QCF_Struct *imu);
//��Ԫ���Ͷ��׵�ͨ�˲��Ľṹ���ָ��
//���������ݣ���λ����ÿ��
//���ٶ����ݣ���λm/s^2
//��ǰʱ�䣬��λ��us
extern void IMU_QCF_Update(IMU_QCF_Struct *imu, float gx, float gy, float gz, float ax, float ay, float az, uint32_t timer_us);
#endif


