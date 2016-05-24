#ifndef _SKYBOARD_PWM_IN_H
#define _SKYBOARD_PWM_IN_H
#include "stm32f10x.h"
extern u16 PWM_IN_BUFFER[6];
void PWM_In_Init(void);
void PWM_In_TIM3_IRQ(void);
void PWM_In_TIM4_IRQ(void);
#endif
