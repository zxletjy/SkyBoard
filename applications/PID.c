//PID运算库
#include "PID.h"

int32_t get_p(int32_t error, SKYBOARD_PID *pid)
{
    return error * pid->kP / 128;
}

int32_t get_i(int32_t error, uint16_t dt, SKYBOARD_PID *pid)
{
    if((pid->kI != 0) && (dt != 0)) {
        pid->integrator += (error * dt / 2048 ) * pid -> kI;
				//积分限幅
				pid->integrator = constrain_int32(pid->integrator, -pid->imax, +pid->imax);		
				
        return pid->integrator / 8192;
    }
    return 0;
}

void reset_I(SKYBOARD_PID *pid)
{
	pid -> integrator = 0;
}

int32_t get_d(int32_t error, uint16_t dt, SKYBOARD_PID *pid)
{
    if ((pid -> kD != 0) && (dt != 0)) {			
			int32_t derivative;
			derivative = error - pid -> last_error; 
			pid -> last_error = error;
			derivative = (derivative * ((uint16_t)0xFFFF / (dt / 16 ))) / 64;
			return (derivative * pid -> kD) / 256;
    }
    return 0;
}

int32_t get_pi(int32_t error, uint16_t dt, SKYBOARD_PID *pid)
{
    return get_p(error, pid) + get_i(error, dt, pid);
}

int32_t get_pid(int32_t error, uint16_t dt, SKYBOARD_PID *pid)
{
    return get_p(error, pid) + get_i(error, dt, pid) + get_d(error, dt, pid);
}

