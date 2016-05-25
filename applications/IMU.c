//飞行器姿态计算
#include "IMU.h"
#include <math.h>

#define GRAVITY         9.80665f    // m/s^2
//欧拉角表示的飞行器姿态
Vector3f angle;

Vector3f Gyro, Acc;
Vector3f EarthAcc;
LPF2ndData_t Acc_lpf_2nd;

float magHold;

float getDeltaT(uint32_t time);
//基于余弦矩阵和互补滤波的姿态解算
//void DCM_CF(Vector3f gyro,Vector3f acc, float deltaT);
//基于四元数和互补滤波的姿态解算
void Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT);

//滤波器参数初始化
void filter_Init(void);
//传感器初始化
void sensor_Init(void);
void normalize(Vector3f *acc);
	
//IMU初始化
void IMU_Init(void)
{
	//滤波器参数初始化
	filter_Init();
}

//更新传感器数据
void updateSensor(void)
{
	//读取加速度
	Read_Acc_Data();
	//读取角速度
	Read_Gyro_Data();	
	//获取角速度，单位为度每秒
	//Gyro = mpu6050.Get_Gyro_in_dps();
	Gyro = Gyro_dps;
	//获取加速度采样值
	//Acc = mpu6050.Get_Acc();
	Acc = Acc_ADC;
}

//计算飞行器姿态
void getAttitude(void)
{
	float deltaT;
	Vector3f Acc_lpf; 
	//加速度数据二阶低通滤波
	Acc_lpf = LPF_2nd(&Acc_lpf_2nd, Acc);

	deltaT = getDeltaT(GetSysTime_us());
	
	Quaternion_CF(Gyro,Acc_lpf,deltaT);
//	IMU_QCF_Update(&IMU_QCF, Gyro.x, Gyro.y, Gyro.z, Acc.x, Acc.y, Acc.z, GetSysTime_us());
//	angle.x = IMU_QCF.roll;
//	angle.y = IMU_QCF.pitch;
//	angle.z = IMU_QCF.yaw;
}
void navUkfRotateVectorByQuat(float *vr, float *v)
{
    float w, x, y, z;

    w = q1;
    x = q2;
    y = q3;
    z = q4;

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}
void GetEarthAcc(Vector3f Acc, Vector3f* earth)
{
	float accIn[3];
	float acc[3];

	accIn[0] = Acc.x;// + UKF_ACC_BIAS_X;
	accIn[1] = Acc.y;// + UKF_ACC_BIAS_Y;
	accIn[2] = Acc.z;// + UKF_ACC_BIAS_Z;

	// rotate acc to world frame
	navUkfRotateVectorByQuat(acc, accIn);
	acc[2] += GRAVITY;
	
	earth->x = acc[0];
	earth->y = acc[1];
	earth->z = acc[2];
}

#define Kp 2.0f        //加速度权重，越大则向加速度测量值收敛越快
#define Ki 0.001f      //误差积分增益

//四元数更新姿态
void Quaternion_CF(Vector3f gyro,Vector3f acc, float deltaT)
{
	Vector3f V_gravity = {0,0,0}, V_error = {0,0,0}, V_error_I = {0,0,0};
	
	//重力加速度归一化
	//acc.normalize();
	normalize(&acc);
	//提取四元数的等效余弦矩阵中的重力分量
	vector_gravity(&V_gravity);
	
	//向量叉积得出姿态误差
	//V_error = acc % V_gravity;
	V_error.x = acc.y * V_gravity.z - acc.z * V_gravity.y;
	V_error.y = acc.z * V_gravity.x - acc.x * V_gravity.z;
	V_error.z = acc.x * V_gravity.y - acc.y * V_gravity.x;
	//对误差进行积分	
	//V_error_I += V_error * Ki;
	V_error_I.x += V_error.x * Ki;
	V_error_I.y += V_error.y * Ki;
	V_error_I.z += V_error.z * Ki;
	//互补滤波，姿态误差补偿到角速度上，修正角速度积分漂移
	//gyro += V_error * Kp + V_error_I;		
	gyro.x += V_error.x * Kp + V_error_I.x;
	gyro.y += V_error.y * Kp + V_error_I.y;
	gyro.z += V_error.z * Kp + V_error_I.z;
	//一阶龙格库塔法更新四元数
	Runge_Kutta_1st(&gyro, deltaT);
	
	//四元数归一化
	Q_normalize();
	
	//四元数转欧拉角
	to_euler(&angle.x, &angle.y, &angle.z);
	
	GetEarthAcc(acc, &EarthAcc);
}

void filter_Init()
{
	//加速度二阶低通滤波器系数计算
	LPF_2nd_Factor_Cal(&Acc_lpf_2nd);
}

void sensor_Init()
{
	//初始化MPU6050，1Khz采样率，42Hz低通滤波
	MPU6050_Init(1000,42);
}

float getDeltaT(uint32_t currentT)
{
	static uint32_t previousT;
	float	deltaT = (currentT - previousT) * 1e-6;	
	previousT = currentT;
	
	return deltaT;
}
void normalize(Vector3f *acc)
{
	float r;
	r = pythagorous3(acc->x, acc->y, acc->z);
	acc->x = acc->x / r;
	acc->y = acc->y / r;
	acc->z = acc->z / r;
}
