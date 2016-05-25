//�������
#include "Motor.h"

uint16_t motorPWM[4];	

void writeMotor(uint16_t throttle, int32_t pidTermRoll, int32_t pidTermPitch, int32_t pidTermYaw)
{
	u8 i=0;
	//����X��
	motorPWM[0] = throttle - pidTermRoll + pidTermPitch - pidTermYaw; //����		3
	motorPWM[1] = throttle - pidTermRoll - pidTermPitch + pidTermYaw; //ǰ��		2
	motorPWM[2] = throttle + pidTermRoll + pidTermPitch + pidTermYaw; //����		4
	motorPWM[3] = throttle + pidTermRoll - pidTermPitch - pidTermYaw; //ǰ��		1
	
	for (i = 0; i < 4; i++) 
		motorPWM[i] = constrain_uint16(motorPWM[i], MINTHROTTLE, MAXTHROTTLE);

	//���δ�������򽫵���������Ϊ���
	if(!ARMED || rc.rawData[THROTTLE] < 1200)	
		for(i=0; i< 4 ; i++)
			motorPWM[i] = 1000;

	//д����PWM
	SetPwm(motorPWM);
	
}

void getPWM(uint16_t* pwm)
{
	*(pwm) = motorPWM[0];
	*(pwm+1) = motorPWM[1];
	*(pwm+2) = motorPWM[2];
	*(pwm+3) = motorPWM[3];
}


