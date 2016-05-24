#ifndef __SKYBOARD_FLYCONTROL_H
#define __SKYBOARD_FLYCONTROL_H

#include "Config.h"

#define FLYANGLE_MAX 450  //���������45��

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
//��̬�⻷����
void Attitude_Outter_Loop(void);
//��̬�ڻ�����
void Attitude_Inner_Loop(void);
#endif
