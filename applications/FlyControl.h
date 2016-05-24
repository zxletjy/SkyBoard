#ifndef __SKYBOARD_FLYCONTROL_H
#define __SKYBOARD_FLYCONTROL_H

#include "Config.h"

#define FLYANGLE_MAX 450  //最大飞行倾角45度

enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDLEVEL,
    PIDMAG,
		PIDITEMS
};

typedef struct
{
	SKYBOARD_PID pid[PIDITEMS];
	uint8_t yawRate;
	int32_t RateError[3];
}SKYBOARD_FlyControl;
extern SKYBOARD_FlyControl fc;
void Fc_Init(void);
void PID_Reset(void);
//姿态外环控制
void Attitude_Outter_Loop(void);
//姿态内环控制
void Attitude_Inner_Loop(void);
#endif
