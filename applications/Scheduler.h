#ifndef __SKYBOARD_SCHEDULER_H
#define __SKYBOARD_SCHEDULER_H

#include "board.h"
#include "Config.h"

typedef struct
{
	//任务时基计数变量
	uint16_t cnt_1ms,cnt_2ms,cnt_5ms,cnt_10ms,cnt_20ms;

}SKYBOARD_Scheduler;

extern SKYBOARD_Scheduler scheduler;

void SKYBOARD_Loop(void);
#endif











