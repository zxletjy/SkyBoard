//***********************************************************
//									串口
//***********************************************************
#include "usart1.h"
#include "dt.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
//**************************************************************************************************************************
//串口1函数
////************************************************************************************************************************
//USART1初始化
void USART1_Init(uint32_t rate)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1,ENABLE);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_DeInit(USART1);
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate=rate;
	USART_Init(USART1,&USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	//使能USART1接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

}

void USART1_Sendu8Package(u8 *p,u8 num)
{
	u8 i;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(i=0;i<num;i++)
	{
		USART1->DR=*p;
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		p++;
	}
}
void USART1_Sendu16Package(u16 *p,u8 num)
{
	u8 i=0;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(i=0;i<num;i++)
	{
		USART1->DR=BYTE0(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		USART1->DR=BYTE1(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		p++;
	}
}
void USART1_Sendu32Package(u32 *p,u8 num)
{
	u8 i=0;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(i=0;i<num;i++)
	{
		USART1->DR=BYTE0(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		USART1->DR=BYTE1(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		USART1->DR=BYTE2(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		USART1->DR=BYTE3(*p);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
		USART_ClearFlag(USART1,USART_FLAG_TC);
		p++;
	}
}
u8 TxBuffer[256];
u8 TxCounter=0;
u8 count=0;
static u8 RxBuffer[50];
static u8 RxState = 0;
//串口1中断服务子程序
void USART1_IRQ(void)
{
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//??!????if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)???
    {
        USART_ReceiveData(USART1);
    }
		
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))//if(USART_GetITStatus(USART1,USART_IT_TXE)!=RESET)
	{
		USART1->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
		if(TxCounter == count)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE中断
			//USART_ITConfig(USART1,USART_IT_TXE,DISABLE);
		}
	}
//接收中断 (接收寄存器非空) 
	if(USART1->SR & (1<<5))//if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART1->DR;
		static u8 _data_len = 0,_data_cnt = 0;
		if(RxState==0&&com_data==0xAA)
		{
			RxState=1;
			RxBuffer[0]=com_data;
		}
		else if(RxState==1&&com_data==0xAF)
		{
			RxState=2;
			RxBuffer[1]=com_data;
		}
		else if(RxState==2&&com_data>0&&com_data<0XF1)
		{
			RxState=3;
			RxBuffer[2]=com_data;
		}
		else if(RxState==3&&com_data<50)
		{
			RxState = 4;
			RxBuffer[3]=com_data;
			_data_len = com_data;
			_data_cnt = 0;
		}
		else if(RxState==4&&_data_len>0)
		{
			_data_len--;
			RxBuffer[4+_data_cnt++]=com_data;
			if(_data_len==0)
				RxState = 5;
		}
		else if(RxState==5)
		{
			RxState = 0;
			RxBuffer[4+_data_cnt]=com_data;
			Data_Receive_Anl(RxBuffer,_data_cnt+5);
		}
		else
			RxState = 0;
	}
}
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
		TxBuffer[count++] = *(DataToSend+i);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}
