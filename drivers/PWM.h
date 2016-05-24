#ifndef __SKYBOARD_DRV_PWM_H__
#define __SKYBOARD_DRV_PWM_H__

#include "board.h"

#define MAXMOTORS 4

void PWM_Out_Init(uint16_t hz);
void SetPwm(uint16_t pwm[MAXMOTORS]);
#endif

