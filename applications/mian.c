#include "Config.h"

int main(void)
{
	//��ʼ���ɿذ��Ӳ������
	SkyBoard_Init();
	//��ʼ������
	Param_Init();
	
	//��ʼ��IMU�����Բ�����Ԫ��
	IMU_Init();	
	
	while(1)
	{
		SKYBOARD_Loop();
	}
}

