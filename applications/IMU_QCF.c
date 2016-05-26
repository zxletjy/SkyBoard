#include "IMU_QCF.h"
IMU_QCF_Struct IMU_QCF;
//IMU初始化
void IMU_QCF_Init(IMU_QCF_Struct *imu)
{
	//四元数初始化
	imu->Q1 = 1;
	imu->Q2 = 0;
	imu->Q3 = 0;
	imu->Q4 = 0;
	//截止频率:30Hz 采样频率:500Hz
	imu->Acc_LPF2nd.b0 = 0.1883633f;
	imu->Acc_LPF2nd.a1 = 1.023694f;
	imu->Acc_LPF2nd.a2 = 0.2120577f;
	//
	imu->ki = 2.0f;
	imu->kp = 0.001f;
}
//四元数和二阶低通滤波的结构体的指针
//陀螺仪数据，单位弧度每秒
//加速度数据，直接是原始数据进来
//当前时间，单位是us
void IMU_QCF_Update(IMU_QCF_Struct *imu, float gx, float gy, float gz, float ax, float ay, float az, uint32_t timer_us)
{
	static uint32_t previousT = 0;
	float dt;
	float acc_x = 0, acc_y = 0, acc_z = 0;
	float V_gravity_x = 0, V_gravity_y = 0, V_gravity_z = 0;
	float V_error_x = 0, V_error_y = 0, V_error_z = 0;
	float V_error_I_x = 0, V_error_I_y = 0, V_error_I_z = 0;
	float r = 0;
	
	acc_x = ax * imu->Acc_LPF2nd.b0 + imu->Acc_LPF2nd.lastout_x * imu->Acc_LPF2nd.a1 - imu->Acc_LPF2nd.preout_x * imu->Acc_LPF2nd.a2;
	acc_y = ay * imu->Acc_LPF2nd.b0 + imu->Acc_LPF2nd.lastout_y * imu->Acc_LPF2nd.a1 - imu->Acc_LPF2nd.preout_y * imu->Acc_LPF2nd.a2;
	acc_z = az * imu->Acc_LPF2nd.b0 + imu->Acc_LPF2nd.lastout_z * imu->Acc_LPF2nd.a1 - imu->Acc_LPF2nd.preout_z * imu->Acc_LPF2nd.a2;
	
	imu->Acc_LPF2nd.preout_x = imu->Acc_LPF2nd.lastout_x;
	imu->Acc_LPF2nd.preout_y = imu->Acc_LPF2nd.lastout_y;
	imu->Acc_LPF2nd.preout_z = imu->Acc_LPF2nd.lastout_z;
	
	imu->Acc_LPF2nd.lastout_x = acc_x;
	imu->Acc_LPF2nd.lastout_y = acc_y;
	imu->Acc_LPF2nd.lastout_z = acc_z;
	
	
	dt = (timer_us - previousT) * 1e-6;	
	previousT = timer_us;
	
	//加速度计数据归一化
	r = sqrtf(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z);
	acc_x/=r;
	acc_y/=r;
	acc_z/=r;
	//提取四元数的等效余弦矩阵中的重力分量
	V_gravity_x = 2*(imu->Q2*imu->Q4 - imu->Q1*imu->Q3);								
  V_gravity_y = 2*(imu->Q1*imu->Q2 + imu->Q3*imu->Q4);						  
  V_gravity_z = 1-2*(imu->Q2*imu->Q2 + imu->Q3*imu->Q3);
	
	//向量叉积得出姿态误差
	V_error_x = acc_y * V_gravity_z - acc_z * V_gravity_y;
	V_error_y = acc_z * V_gravity_x - acc_x * V_gravity_z;
	V_error_z = acc_x * V_gravity_y - acc_y * V_gravity_x;
	
	//对误差进行积分	
	V_error_I_x += V_error_x * imu->ki;
	V_error_I_y += V_error_y * imu->ki;
	V_error_I_z += V_error_z * imu->ki;
	
	//互补滤波，姿态误差补偿到角速度上，修正角速度积分漂移
	gx += V_error_x * imu->kp + V_error_I_x;
	gy += V_error_y * imu->kp + V_error_I_y;
	gz += V_error_z * imu->kp + V_error_I_z;
	imu->gyro_cf_x = gx;
	imu->gyro_cf_y = gy;
	imu->gyro_cf_z = gz;
	
	//一阶龙格库塔法更新四元数
	imu->Q1 += 0.5 * (-imu->Q2*gx - imu->Q3*gy - imu->Q4*gz)* dt;
  imu->Q2 += 0.5 * (imu->Q1*gx + imu->Q3*gz - imu->Q4*gy)* dt;
  imu->Q3 += 0.5 * (imu->Q1*gy - imu->Q2*gz + imu->Q4*gx)* dt;
  imu->Q4 += 0.5 * (imu->Q1*gz + imu->Q2*gy - imu->Q3*gx)* dt;
	
	//四元数归一化
	r = sqrtf(imu->Q1*imu->Q1 + imu->Q2*imu->Q2 + imu->Q3*imu->Q3 + imu->Q4*imu->Q4);
	imu->Q1/=r;
	imu->Q2/=r;
	imu->Q3/=r;
	imu->Q4/=r;
	//四元数转欧拉角

	imu->roll_r = atan2f(2.0f*(imu->Q1*imu->Q2 + imu->Q3*imu->Q4),1 - 2.0f*(imu->Q2*imu->Q2 + imu->Q3*imu->Q3));
	imu->roll = imu->roll_r * RAD_TO_DEG;

	imu->pitch = 2.0f*(imu->Q1*imu->Q3 - imu->Q2*imu->Q4);
	// 使用safe_asin()来处理pitch接近90/-90时的奇点
	if (isnan(imu->pitch)) {
		imu->pitch = 0.0;
	}
	if (imu->pitch >= 1.0f) {
			imu->pitch = M_PI/2;
	}
	if (imu->pitch <= -1.0f) {
			imu->pitch = -M_PI/2;
	}
	imu->pitch_r = asinf(imu->pitch);
	imu->pitch = imu->pitch_r * RAD_TO_DEG;

	imu->yaw_r = atan2f(2.0f*(imu->Q2*imu->Q3 - imu->Q1*imu->Q4), 2.0f*(imu->Q1*imu->Q1 + imu->Q2*imu->Q2) - 1);
	imu->yaw = imu->yaw_r * RAD_TO_DEG;
}

