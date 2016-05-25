#include "Config.h"

int main(void)
{
	//初始化飞控板的硬件设置
	SkyBoard_Init();
	//初始化参数
	Param_Init();
	
	//初始化IMU（惯性测量单元）
	IMU_Init();	
	
	while(1)
	{
		SKYBOARD_Loop();
	}
}

