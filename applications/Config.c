#include "Config.h"

uint8_t ARMED = 0;

//Ö¸Ê¾µÆ
void Pilot_Light(void)
{
	static u8 cnt = 0;
	
	if(ARMED)
	{
		if(Acc_CALIBRATED == 0)
		{
			cnt++;
			switch(cnt)
			{
				case 1:
					LED_ON();
					break;
				case 25:
					LED_OFF();
					break;
				case 50:
					cnt = 0;
					break;
			}
		}
	}
	else
	{
		LED_ON(); 
	}
	
}
