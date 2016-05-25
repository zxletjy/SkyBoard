#include "LED.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_LED1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);  
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = LED1_Pin;
	GPIO_Init(GPIO_LED1, &GPIO_InitStructure);
}
//LED1
void LED_ON(void)
{
	GPIO_ResetBits(GPIO_LED1, LED1_Pin);	
}

void LED_OFF(void)
{
	GPIO_SetBits(GPIO_LED1, LED1_Pin);		
}
