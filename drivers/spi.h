#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f10x.h"

void SPI1_Init(void);
u8 SPI1_RW(u8 dat);
void SPI1_CE_H(void);
void SPI1_CE_L(void);
void SPI1_CSN_H(void);
void SPI1_CSN_L(void);

#endif

