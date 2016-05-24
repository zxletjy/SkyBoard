#ifndef __SKYBOARD_PID_H
#define __SKYBOARD_PID_H

#include <stdint.h>
#include "Math.h"

typedef struct 
{
	//PID����
	uint16_t kP;
	uint16_t kI;
	uint16_t kD;
	//��������
	uint32_t imax;
	//��������ֵ
	int32_t integrator; 
	//��һ�ε��������
	int32_t last_error;
}SKYBOARD_PID;


//���ֿ�������ֵ����
void reset_I(SKYBOARD_PID *pid);
//����PID�����ֵ
int32_t get_pid(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_pi(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_p(int32_t error, SKYBOARD_PID *pid);
int32_t get_i(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_d(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
#endif

