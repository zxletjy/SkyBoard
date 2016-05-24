#ifndef _TJY_USART1_H
#define _TJY_USART1_H
#include "stm32f10x.h"
#define USE_DMA
void USART1_Init(uint32_t rate);	

void USART1_Sendu8Package(u8 *p,u8 num);
void USART1_Sendu16Package(u16 *p,u8 num);
void USART1_Sendu32Package(u32 *p,u8 num);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void USART1_IRQ(void);

#endif
