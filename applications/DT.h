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
//检查是否有接收到无线数据
void Check_Event(void);
//数据发送
void Data_Exchange(void);
//失控保护检查
void Failsafe_Check(void);
#endif









