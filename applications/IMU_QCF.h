#ifndef _IMU_QCF_H_
#define _IMU_QCF_H_
#include <math.h>
#include "stm32f10x.h"
#define M_PI 3.141592653f
#define DEG_TO_RAD 0.01745329f
#define RAD_TO_DEG 57.29577951f
#define GRAVITY        9.80665f    // m/s^2
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
	float pitch;//单位是度
	float roll;
	float yaw;
	
	float pitch_r;//单位是弧度
	float roll_r;
	float yaw_r;
	
	LPF2nd_Struct Acc_LPF2nd;
	
	float Q1;	//四元数
	float Q2;
	float Q3;
	float Q4;
	
	float ki;//加速度权重，越大则向加速度测量值收敛越�
	float kp;//误差积分增益
	
	float gyro_cf_x;	//修正之后数据
	float gyro_cf_y;
	float gyro_cf_z;
	
}IMU_QCF_Struct;

extern IMU_QCF_Struct IMU_QCF;

extern void IMU_QCF_Init(IMU_QCF_Struct *imu);
//四元数和二阶低通滤波的结构体的指针
//陀螺仪数据，单位弧度每秒
//加速度数据，，直接是原始数据进来
//当前时间，单位是us
extern void IMU_QCF_Update(IMU_QCF_Struct *imu, float gx, float gy, float gz, float ax, float ay, float az, uint32_t timer_us);
#endif


