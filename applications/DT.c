//无线数据收发和处理
#include "DT.h"
#include "usb_int.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

SKYBOARD_DT dt;
u8 data_to_send[50];
void Send_BAT(void);
void Send_Status(void);
void Send_Senser(void);
void Send_RCData(void);
void Send_MotoPWM(void);
void Send_PID1(void);
void Send_PID2(void);
void Send_Check(u16 check);

void Send_Data(u8 *dataToSend , u8 length);

void Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	u8 i;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头

/////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			Acc_CALIBRATED = 1;
		if(*(data_buf+4)==0X02)
			Gyro_CALIBRATED = 1;
		if(*(data_buf+4)==0X03)
		{
			Acc_CALIBRATED = 1;		
			Gyro_CALIBRATED = 1;			
		}
	}
	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			Send_PID1();		
			Send_PID2();
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
	}
/*
	if(*(data_buf+2)==0X03)
	{
		rc.rawData[THROTTLE] = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
		rc.rawData[YAW] = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
		rc.rawData[ROLL] = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		rc.rawData[PITCH] = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
	}
*/
	if(*(data_buf+2)==0X10)								//PID1
	{
		fc.pid[PIDROLL].kP = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
		fc.pid[PIDROLL].kI = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
		fc.pid[PIDROLL].kD = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		fc.pid[PIDPITCH].kP = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
		fc.pid[PIDPITCH].kI = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
		fc.pid[PIDPITCH].kD = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
		fc.pid[PIDYAW].kP = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
		fc.pid[PIDYAW].kI = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
		fc.pid[PIDYAW].kD = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X11)								//PID2
	{
		fc.pid[PIDALT].kP = (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
		fc.pid[PIDALT].kI = (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
		fc.pid[PIDALT].kD = (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		fc.pid[PIDLEVEL].kP = (vs16)(*(data_buf+10)<<8)|*(data_buf+11);
		fc.pid[PIDLEVEL].kI = (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
		fc.pid[PIDLEVEL].kD = (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
		fc.pid[PIDMAG].kP = (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
		fc.pid[PIDMAG].kI = (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
		fc.pid[PIDMAG].kD = (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID3
	{
		Send_Check(sum);
		SAVE_PID();
	}
	if(*(data_buf+2)==0X12)								//PID4
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID5
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X12)								//PID6
	{
		Send_Check(sum);
	}
	if(*(data_buf+2)==0X16)								//OFFSET
	{

	}

/////////////////////////////////////////////////////////////////////////////////////////////////
	if(*(data_buf+2)==0x18)					
	{

	}
}

void Check_Event(void)
{

}



void Send_Status(void)
{
	u8 _cnt=0;
	u8 i;
	vs16 _temp;
	vs32 _temp2 = 0;//UltraAlt * 100;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(IMU_QCF.roll*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(IMU_QCF.pitch*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(IMU_QCF.yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}


void Send_Senser(void)
{
	u8 _cnt=0;
	u8 i;
	u8 sum = 0;
	vs16 _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	_temp = Acc.x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Acc.y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = Acc.z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//_temp = mpu6050.Get_Gyro().x;	
	_temp = Gyro_ADC.x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//_temp = mpu6050.Get_Gyro().y;	
	_temp = Gyro_ADC.y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//_temp = mpu6050.Get_Gyro().z;	
	_temp = Gyro_ADC.z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	data_to_send[_cnt++] = 0;
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_RCData(void)
{
	u8 _cnt=0;
	u8 i;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(rc.rawData[THROTTLE]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[THROTTLE]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[YAW]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[YAW]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[ROLL]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[ROLL]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[PITCH]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[PITCH]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX1]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX1]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX2]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX2]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX3]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX3]);
	data_to_send[_cnt++]=BYTE1(rc.rawData[AUX4]);
	data_to_send[_cnt++]=BYTE0(rc.rawData[AUX4]);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_MotoPWM(void)
{
	u8 _cnt=0;
	u8 i;
	uint16_t Moto_PWM[6];
	u8 sum = 0;
	getPWM(Moto_PWM);
	
	for(i=0;i<6;i++)
		Moto_PWM[i] -= 1000;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[1]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[3]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[3]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[5]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[5]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[2]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[0]);
	data_to_send[_cnt++]=BYTE1(Moto_PWM[4]);
	data_to_send[_cnt++]=BYTE0(Moto_PWM[4]);

	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_PID1(void)
{
	u8 _cnt=0;
	u8 i;
	vs16 _temp;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10;
	data_to_send[_cnt++]=0;
	
	
	_temp = fc.pid[PIDROLL].kP ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDROLL].kI ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDROLL].kD ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDPITCH].kP ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDPITCH].kI ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDPITCH].kD ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDYAW].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDYAW].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDYAW].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	Send_Data(data_to_send, _cnt);
}

void Send_PID2(void)
{
	u8 _cnt=0;
	u8 i;
	vs16 _temp;
	u8 sum = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x11;
	data_to_send[_cnt++]=0;

	
	_temp = fc.pid[PIDALT].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDALT].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDALT].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDLEVEL].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDLEVEL].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDLEVEL].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDMAG].kP;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDMAG].kI;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = fc.pid[PIDMAG].kD;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}

void Send_Check(u16 check)
{
	u8 i;
	u8 sum = 0;
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=BYTE1(check);
	data_to_send[6]=BYTE0(check);
	
	
	for(i=0;i<7;i++)
		sum += data_to_send[i];
	
	data_to_send[7]=sum;

	Send_Data(data_to_send, 8);
}

void Send_Data(u8 *dataToSend , u8 length)
{
	//Uart1_Put_Buf(dataToSend, length);
	USB_Virtual_Serial_Send(dataToSend, length);
}

void Send_DEBUG(void)
{
	u8 _cnt=0;
	u8 i;
	vs16 _temp;
	u8 sum = 0;
	float data = 0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	
	data = Acc.z;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data = IMU_QCF.Acc_LPF2nd.lastout_z;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data = IMU_QCF.yaw;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data = EarthAcc.x;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data = EarthAcc.y;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data = EarthAcc.z;
	data_to_send[_cnt++]=BYTE3(data);
	data_to_send[_cnt++]=BYTE2(data);
	data_to_send[_cnt++]=BYTE1(data);
	data_to_send[_cnt++]=BYTE0(data);
	
	data_to_send[3] = _cnt-4;
	
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Send_Data(data_to_send, _cnt);
}
void Data_Exchange(void)
{
	static int cnt = 0;
	
	switch(cnt)
	{
		case SEND_STATES:
			Send_Status();
			cnt++;
			break;
		case SEND_SENSER: 
			Send_Senser();
			cnt++;
			break;
		case SEND_PID1:
			Send_PID1();		
			cnt++;
			break;
		case SEND_PID2:
			Send_PID2();			
			cnt++;
			break;
		case SEND_RC:
			Send_RCData();
			cnt++;
			break;
		case SEND_MOTORPWM:
			Send_MotoPWM();
			cnt++;
			break;
		case SEND_DEBUG:
			Send_DEBUG();
			cnt++;
			break;
		default:
			cnt=0;
			break;
	}
}

void Failsafe_Check(void)
{
	if(ARMED == 1)//如果已经解锁了
	{
		if(rc.rawData[THROTTLE] <= 1200)
		{
			dt.FailCheck++;
		}
		else
		{
			dt.FailCheck = 0;
		}
		if(dt.FailCheck >= 1000)//20ms * 100 = 2000ms=2s
		{
			ARMED = 0;//上锁
			dt.FailCheck = 0;
		}
	}
	else
	{
		if((rc.rawData[THROTTLE] <= 1200)&&(rc.rawData[YAW] >= 1600))
		{
			dt.FailCheck++;
			if(dt.FailCheck >= 10)//20ms * 100 = 2000ms=2s
			{
				ARMED = 1;//解锁
				dt.FailCheck = 0;
			}
		}
		else if((rc.rawData[THROTTLE] <= 1200)&&(rc.rawData[YAW] <= 1400))
		{
			dt.FailCheck++;
			if(dt.FailCheck >= 100)//20ms * 100 = 2000ms=2s
			{
				ARMED = 1;//解锁
				Acc_CALIBRATED = 1;
				Gyro_CALIBRATED = 1;
				dt.FailCheck = 0;
			}
		}
		else
		{
			dt.FailCheck = 0;
		}
	}
}
