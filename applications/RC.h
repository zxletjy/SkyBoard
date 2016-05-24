#ifndef __SKYBOARD_RC_H
#define __SKYBOARD_RC_H

#include "Config.h"

#define RC_MID  			1500                     
#define RC_MINCHECK		1150                      
#define RC_MAXCHECK  	1850 

enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};

typedef struct
{
	uint16_t rawData[8];
	int16_t Command[4];
}SKYBOARD_RC;

extern SKYBOARD_RC rc;
//ң��ͨ�����ݴ���
void Cal_Command(void);
//ҡ��λ�ü��
void check_sticks(void);
#endif



