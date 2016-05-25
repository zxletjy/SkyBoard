#include "ultran.h"
void ULTRAN_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  
  /* ≈‰÷√÷–∂œ‘¥ */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
