//������
#include "Scheduler.h"
#include "usb_int.h"
SKYBOARD_Scheduler scheduler;


static void SKYBOARD_Loop_1000Hz(void)	//1msִ��һ��
{
	u8 data[32],length = 0;
	rc.rawData[THROTTLE] = PWM_IN_BUFFER[2];
	rc.rawData[ROLL] = PWM_IN_BUFFER[3];
	rc.rawData[YAW] = PWM_IN_BUFFER[0];
	rc.rawData[PITCH] = PWM_IN_BUFFER[1];
	
	USB_Virtual_Serial_Receive(data, &length);
	Data_Receive_Anl(data, length);
}

static void SKYBOARD_Loop_500Hz(void)	//2msִ��һ��
{	
	//���´���������	
	updateSensor();		
	
	//�����������̬
	getAttitude();
	
	//��������̬�ڻ�����
	Attitude_Inner_Loop();	
}

static void SKYBOARD_Loop_200Hz(void)	//5msִ��һ��
{
	//��������̬�⻷����
	Attitude_Outter_Loop();	
}

static void SKYBOARD_Loop_100Hz(void)	//10msִ��һ��
{
	//���ͷ���������
	Data_Exchange();
}

static void SKYBOARD_Loop_50Hz(void)	//20msִ��һ��
{
	//ң��ͨ�����ݴ���
	Cal_Command();
	
	//ҡ��λ�ü��
	check_sticks();
	
	//ʧ�ر������
	Failsafe_Check();
	
	//LEDָʾ�ƿ���
	Pilot_Light();
}

void SKYBOARD_Loop(void)
{
	if(scheduler.cnt_1ms >= 1){
		SKYBOARD_Loop_1000Hz();	
		scheduler.cnt_1ms = 0;
	}
	if(scheduler.cnt_2ms >= 2){
		SKYBOARD_Loop_500Hz();
		scheduler.cnt_2ms = 0;
	}		
	if(scheduler.cnt_5ms >= 5){	
		SKYBOARD_Loop_200Hz();
		scheduler.cnt_5ms = 0;
	}
	if(scheduler.cnt_10ms >= 10){		
		SKYBOARD_Loop_100Hz();
		scheduler.cnt_10ms = 0;
	}
	if(scheduler.cnt_20ms >= 20){		
		SKYBOARD_Loop_50Hz();
		scheduler.cnt_20ms = 0;
	}
}
