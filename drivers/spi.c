#include "stm32f10x.h"
#include "board.h"

void SPI1_Init(void)
{
	SPI_InitTypeDef SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_GPIO_SPI, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_GPIO_CE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); 

	/*���� SPI_NRF_SPI�� SCK,MISO,MOSI���� */ 
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_SCK| SPI_Pin_MISO| SPI_Pin_MOSI; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù��� 
	GPIO_Init(SKYBOARD_GPIO_SPI, &GPIO_InitStructure);
	/*����SPI_NRF_SPI��CE���ţ���SPI_NRF_SPI�� CSN ����:*/
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CE; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SKYBOARD_GPIO_CE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_Pin_CSN; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
	GPIO_Init(SKYBOARD_GPIO_SPI, &GPIO_InitStructure);	
	
	GPIO_SetBits(SKYBOARD_GPIO_SPI, SPI_Pin_CSN);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //˫��ȫ˫�� 
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //��ģʽ 
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; //���ݴ�С8λ 
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //ʱ�Ӽ��ԣ�����ʱΪ�� 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //��1��������Ч��������Ϊ����ʱ�� 
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //NSS�ź���������� 
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //8��Ƶ��9MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; //��λ��ǰ 
	SPI_InitStructure.SPI_CRCPolynomial = 7; 
	SPI_Init(SPI1, &SPI_InitStructure); 
	/* Enable SPI1 */ 
	SPI_Cmd(SPI1, ENABLE);
	
  /* EXTI line(PC13) mode config */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line15;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}
u8 SPI1_RW(u8 dat)
{
	/* �� SPI���ͻ������ǿ�ʱ�ȴ� */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); 
	/* ͨ�� SPI2����һ�ֽ����� */ 
	SPI_I2S_SendData(SPI1, dat); 
	/* ��SPI���ջ�����Ϊ��ʱ�ȴ� */ 
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); 
	/* Return the byte read from the SPI bus */ 
	return SPI_I2S_ReceiveData(SPI1); 
}
void SPI1_CE_H(void)
{
	GPIO_SetBits(SKYBOARD_GPIO_CE, SPI_Pin_CE);
}
void SPI1_CE_L(void)
{
	GPIO_ResetBits(SKYBOARD_GPIO_CE, SPI_Pin_CE);
}
void SPI1_CSN_H(void)
{
	GPIO_SetBits(SKYBOARD_GPIO_SPI, SPI_Pin_CSN);
}
void SPI1_CSN_L(void)
{
	GPIO_ResetBits(SKYBOARD_GPIO_SPI, SPI_Pin_CSN);
}


