#ifndef __SKYBOARD_PID_H
#define __SKYBOARD_PID_H

#include <stdint.h>
#include "Math.h"

typedef struct 
{
	//PID参数
	uint16_t kP;
	uint16_t kI;
	uint16_t kD;
	//积分上限
	uint32_t imax;
	//积分器的值
	int32_t integrator; 
	//上一次的误差输入
	int32_t last_error;
}SKYBOARD_PID;


//积分控制器的值清零
void reset_I(SKYBOARD_PID *pid);
//返回PID计算的值
int32_t get_pid(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_pi(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_p(int32_t error, SKYBOARD_PID *pid);
int32_t get_i(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
int32_t get_d(int32_t error, uint16_t dt, SKYBOARD_PID *pid);
#endif

