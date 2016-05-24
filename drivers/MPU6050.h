#ifndef __SKYBOARD_DRV_MPU6050_H
#define	__SKYBOARD_DRV_MPU6050_H

#include "board.h"

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400

extern u8 Acc_CALIBRATED;
extern u8 Gyro_CALIBRATED;
extern Vector3i Acc_Offset,Gyro_Offset;
extern Vector3f Acc_ADC,Gyro_ADC;
extern Vector3f Gyro_dps;

//初始化6050
void MPU6050_Init(uint16_t sample_rate, uint16_t lpf);
//读取加速度
void Read_Acc_Data(void);
//读取角速度
void Read_Gyro_Data(void);
//加速度零偏矫正
void CalOffset_Acc(void);
//陀螺仪零偏矫正
void CalOffset_Gyro(void);

#endif








