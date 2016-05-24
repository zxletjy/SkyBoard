#ifndef __SKYBOARD_DT_H
#define __SKYBOARD_DT_H

#include "Config.h"

typedef struct
{
	uint16_t FailCheck;
	u8 Send_Status;
	u8 Send_Senser;
	u8 Send_PID1;
	u8 Send_PID2;
	u8 Send_PID3;
	u8 Send_RCData;
	u8 Send_Offset;
	u8 Send_MotoPwm;
	u8 Send_Bat;
}SKYBOARD_DT;

void Data_Receive_Anl(u8 *data_buf,u8 num);
//����Ƿ��н��յ���������
void Check_Event(void);
//���ݷ���
void Data_Exchange(void);
//ʧ�ر������
void Failsafe_Check(void);
#endif









