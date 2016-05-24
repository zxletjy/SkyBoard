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
#define IMU_LOOP_TIME					2000	//��λΪuS
#define PID_INNER_LOOP_TIME		2000	//��λΪus
#define PID_OUTER_LOOP_TIME		5000	//��λΪus

#define ACC_1G 			4096		//�ɼ��ٶȼƵ�����ȷ��
#define ACC_LPF_CUT 10.0f		//���ٶȵ�ͨ�˲�����ֹƵ��10Hz

#define GYRO_CF_TAU 1.2f
/*---------------------------------------------*/

/*--------------------------------------------------------*/

extern uint8_t ARMED;

void Pilot_Light(void);
#endif

