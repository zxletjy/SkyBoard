//飞行控制
#include "FlyControl.h"

SKYBOARD_FlyControl fc;


void Fc_Init(void)
{
	fc.yawRate = 120;
	//重置PID参数
	PID_Reset();
}

//重置PID参数
void PID_Reset(void)
{
	fc.pid[PIDROLL].kP = 65;
	fc.pid[PIDROLL].kI = 5;
	fc.pid[PIDROLL].kD = 80;
	fc.pid[PIDROLL].imax = 2000000;
	
	fc.pid[PIDPITCH].kP = 65;
	fc.pid[PIDPITCH].kI = 5;
	fc.pid[PIDPITCH].kD = 80;
	fc.pid[PIDPITCH].imax = 2000000;
	
	fc.pid[PIDYAW].kP = 90;
	fc.pid[PIDYAW].kI = 15;
	fc.pid[PIDYAW].kD = 0;
	fc.pid[PIDYAW].imax = 2000000;
	
	fc.pid[PIDLEVEL].kP = 300;
	fc.pid[PIDLEVEL].kI = 0;
	fc.pid[PIDLEVEL].kD = 0;
	fc.pid[PIDLEVEL].imax = 0;
	
	fc.pid[PIDMAG].kP = 15;
	fc.pid[PIDMAG].kI = 0;
	fc.pid[PIDMAG].kD = 0;
	fc.pid[PIDMAG].imax = 0;
}

//飞行器姿态外环控制
void Attitude_Outter_Loop(void)
{
	int32_t	errorAngle[2];
	Vector3f Gyro_this = {0,0,0};
	
	//计算角度误差值
	errorAngle[ROLL] = constrain_int32((rc.Command[ROLL] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - angle.x * 10; 
	errorAngle[PITCH] = constrain_int32((rc.Command[PITCH] * 2) , -((int)FLYANGLE_MAX), +FLYANGLE_MAX) - angle.y * 10; 
	
	//获取角速度
	//Gyro_ADC = mpu6050.Get_Gyro() / 4;
	Gyro_this.x = Gyro_ADC.x / 4;
	Gyro_this.y = Gyro_ADC.y / 4;
	Gyro_this.z = Gyro_ADC.z / 4;
	//得到外环PID输出
	fc.RateError[ROLL] = get_p(errorAngle[ROLL], &fc.pid[PIDLEVEL]) - Gyro_this.x;
	fc.RateError[PITCH] = get_p(errorAngle[PITCH], &fc.pid[PIDLEVEL]) - Gyro_this.y;
	fc.RateError[YAW] = ((int32_t)(fc.yawRate) * rc.Command[YAW]) / 32 - Gyro_this.z;		
}

//飞行器姿态内环控制
void Attitude_Inner_Loop(void)
{
	int32_t PIDTerm[3];
	u8 i;
	for(i=0; i<3;i++)
	{
		//当油门低于检查值时积分清零
		if ((rc.rawData[THROTTLE]) < RC_MINCHECK)	
			reset_I(&fc.pid[i]);
		
		//得到内环PID输出
		PIDTerm[i] = get_pid(fc.RateError[i], PID_INNER_LOOP_TIME, &fc.pid[i]);
	}
	
	PIDTerm[YAW] = -constrain_int32(PIDTerm[YAW], -300 - abs(rc.Command[YAW]), +300 + abs(rc.Command[YAW]));	
		
	//PID输出转为电机控制量
	writeMotor(rc.Command[THROTTLE], PIDTerm[ROLL], PIDTerm[PITCH], PIDTerm[YAW]);
}	

