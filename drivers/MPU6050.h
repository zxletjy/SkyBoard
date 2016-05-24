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

//��ʼ��6050
void MPU6050_Init(uint16_t sample_rate, uint16_t lpf);
//��ȡ���ٶ�
void Read_Acc_Data(void);
//��ȡ���ٶ�
void Read_Gyro_Data(void);
//���ٶ���ƫ����
void CalOffset_Acc(void);
//��������ƫ����
void CalOffset_Gyro(void);

#endif








