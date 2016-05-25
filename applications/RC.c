//遥控通道数据处理
#include "RC.h"

SKYBOARD_RC rc;

void Cal_Command(void)
{
	u8 i;
	for (i = 0; i < 3; i++) 
	{	//处理ROLL,PITCH,YAW这三个轴的数据
		rc.Command[i] = (rc.rawData[i] - RC_MID) * 0.4;
	}	
	rc.Command[YAW] = -rc.Command[YAW];
	rc.Command[THROTTLE] = rc.rawData[THROTTLE];
		
		//-------------------航向锁定------------------
	if (abs(rc.Command[YAW]) < 70 && rc.rawData[THROTTLE] > RC_MINCHECK) 
	{
		int16_t dif = IMU_QCF.yaw - magHold;
		if (dif <= -180)
			dif += 360;
		if (dif >= +180)
			dif -= 360;
		dif = -dif;
		
		rc.Command[YAW] -= dif * fc.pid[PIDMAG].kP * 0.1;  	
	} 	
	else
		magHold = IMU_QCF.yaw;	
		
}


const uint8_t stick_min_flag[4] = {1<<0,1<<2,1<<4,1<<6}; 
const uint8_t stick_max_flag[4] = {1<<1,1<<3,1<<5,1<<7};
#define ROL_L 0x01
#define ROL_H 0x02
#define PIT_L	0x04
#define PIT_H 0x08
#define YAW_L 0x10
#define YAW_H 0x20
#define THR_L 0x40
#define THR_H 0x80

void check_sticks(void)
{
	int i;
	
	static uint8_t stick_flag = 0;

	for (i = 0; i < 4; i++) 
	{
			if(rc.rawData[i]<900||rc.rawData[i]>2000)	//如果摇杆值不在正常范围内，则退出检查
				break;
			
			if (rc.rawData[i] < RC_MINCHECK)
					stick_flag |= stick_min_flag[i];  // check for MIN
			else
					stick_flag &= ~stick_min_flag[i];
			
			if (rc.rawData[i] > RC_MAXCHECK)
					stick_flag |= stick_max_flag[i]; // check for MAX
			else
					stick_flag &= ~stick_max_flag[i];  
	}
}
