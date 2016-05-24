#ifndef __SKYBOARD_MOTOR_H
#define __SKYBOARD_MOTOR_H

#include "Config.h"

#define MINTHROTTLE 1100
#define MAXTHROTTLE 1900

extern uint16_t motorPWM[4];	
void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw);
void getPWM(uint16_t* pwm);
#endif





