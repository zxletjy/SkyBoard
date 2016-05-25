#ifndef __SKYBOARD_DT_H
#define __SKYBOARD_DT_H

#include "Config.h"

typedef struct
{
	uint16_t FailCheck;
}SKYBOARD_DT;
enum
{
	SEND_STATES =0,
	SEND_SENSER,
	SEND_PID1,
	SEND_PID2,
	SEND_RC,
	SEND_MOTORPWM,
	SEND_DEBUG
};
void Data_Receive_Anl(u8 *data_buf,u8 num);
//����Ƿ��н��յ���������
void Check_Event(void);
//���ݷ���
void Data_Exchange(void);
//ʧ�ر������
void Failsafe_Check(void);
#endif









