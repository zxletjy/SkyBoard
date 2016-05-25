//电机控制
#include "Motor.h"

uint16_t motorPWM[4];	

void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	u8 i=0;
	//四轴X型
	motorPWM[0] = throttle - pidTermRoll + pidTermPitch - pidTermYaw; //后右		3
	motorPWM[1] = throttle - pidTermRoll - pidTermPitch + pidTermYaw; //前右		2
	motorPWM[2] = throttle + pidTermRoll + pidTermPitch + pidTermYaw; //后左		4
	motorPWM[3] = throttle + pidTermRoll - pidTermPitch - pidTermYaw; //前左		1
	
	for (i = 0; i < 4; i++) 
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);

	//如果未解锁，则将电机输出设置为最低
	if(!ARMED || rc.rawData[THROTTLE] < 1200)	
		for(i=0; i< 4 ; i++)
			motorPWM[i] = 1000;

	//写入电机PWM
	SetPwm(motorPWM);
	
}

void getPWM(uint16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
}


