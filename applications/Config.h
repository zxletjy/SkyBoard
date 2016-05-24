#ifndef __SKYBOARD_CONFIG_H
#define __SKYBOARD_CONFIG_H

#include "board.h"
#include "PID.h"
#include "Filter.h"
#include "IMU.h"
#include "Scheduler.h"
#include "DT.h"
#include "Motor.h"
#include "RC.h"
#include "FlyControl.h"
#include "Param.h"

/*----------------------IMU--------------------*/
#define IMU_LOOP_TIME					2000	//单位为uS
#define PID_INNER_LOOP_TIME		2000	//单位为us
#define PID_OUTER_LOOP_TIME		5000	//单位为us

#define ACC_1G 			4096		//由加速度计的量程确定
#define ACC_LPF_CUT 10.0f		//加速度低通滤波器截止频率10Hz

#define GYRO_CF_TAU 1.2f
/*---------------------------------------------*/

/*--------------------------------------------------------*/

extern uint8_t ARMED;

void Pilot_Light(void);
#endif

