#include "PWM_In.h"
//u16 ole[6];
u16 PWM_IN_BUFFER[6] = {1500,1500,1500,1500,1500,1500};
void PWM_In_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能TIM3和TIM4时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	//使能GPIOB的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//配置捕获口浮空输入或下拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);
	//复位TIM3
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructure.TIM_Period = 19999;//50Hz
	TIM_TimeBaseStructure.TIM_Prescaler=71;//72MHz/72=1MHz,即每次计数时间为1us
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
	//复位TIM4
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = 19999;//50Hz
	TIM_TimeBaseStructure.TIM_Prescaler=71;//72MHz/72=1MHz,即每次计数时间为1us
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;			//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	//映射到TI1上
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;						//输入不分频
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       									//不滤波
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);
	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void PWM_In_TIM3_IRQ(void)
{
	static u16 temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	if(TIM3->SR & TIM_IT_CC3)
	{
		u16 ccr3 = TIM3->CCR3;
		if(GPIOB->IDR & GPIO_Pin_0)
		{
			temp_cnt3 = ccr3;
			TIM3->CCER |= (1<<9);
		}
		else
		{
			temp_cnt3_2 = ccr3;
			TIM3->CCER &= ~(1<<9);
	//		ole[4]=PWM_IN_BUFFER[4];
			if(temp_cnt3_2>=temp_cnt3)
				PWM_IN_BUFFER[4] = temp_cnt3_2-temp_cnt3;
			else
				PWM_IN_BUFFER[4] = 2000-temp_cnt3+temp_cnt3_2;
	//		if((PWM_IN_BUFFER[4]<1000)||(PWM_IN_BUFFER[4]>2000))PWM_IN_BUFFER[4]=ole[4];
		}
	}
	if(TIM3->SR & TIM_IT_CC4)
	{
		u16 ccr4 = TIM3->CCR4;
		if(GPIOB->IDR & GPIO_Pin_1)
		{
			temp_cnt4 = ccr4;
			TIM3->CCER |= (1<<13);
		}
		else
		{
			temp_cnt4_2 = ccr4;
			TIM3->CCER &= ~(1<<13);
		//	ole[5]=PWM_IN_BUFFER[5];
			if(temp_cnt4_2>=temp_cnt4)
				PWM_IN_BUFFER[5] = temp_cnt4_2-temp_cnt4;
			else
				PWM_IN_BUFFER[5] = 2000-temp_cnt4+temp_cnt4_2;
		//	if((PWM_IN_BUFFER[5]<1000)||(PWM_IN_BUFFER[5]>2000))PWM_IN_BUFFER[5]=ole[5];
		}
	}
}
void PWM_In_TIM4_IRQ(void)
{
	static u16 temp_cnt1,temp_cnt1_2,temp_cnt2,temp_cnt2_2,temp_cnt3,temp_cnt3_2,temp_cnt4,temp_cnt4_2;
	if(TIM4->SR & TIM_IT_CC1)//如果是捕获口1中断
	{
		u16 ccr1 = TIM4->CCR1;	//将捕获值保存在ccr1
		TIM4->SR &= ~TIM_FLAG_CC1OF;//清除捕获口1中断标志位
		if(GPIOB->IDR & GPIO_Pin_6)	//如果是上升沿捕获
		{
			temp_cnt1 = ccr1;					//将ccr1中的值保存为第一次捕获值
			TIM4->CCER |= (1<<1);			//转化为下降沿触发
		}
		else//如果是下降沿捕获
		{
			temp_cnt1_2 = ccr1;				//将ccr1中的值保存为第二次捕获值
			TIM4->CCER &= ~(1<<1);		//转化为上升沿触发
			//处理数据
	//		ole[0]=PWM_IN_BUFFER[0];
			if(temp_cnt1_2>=temp_cnt1)
				PWM_IN_BUFFER[0] = temp_cnt1_2-temp_cnt1;
			else
				PWM_IN_BUFFER[0] = 2000-temp_cnt1+temp_cnt1_2;
	//		if((PWM_IN_BUFFER[0]<1000)||(PWM_IN_BUFFER[0]>2000))PWM_IN_BUFFER[0]=ole[0];
		}
	}
	if(TIM4->SR & TIM_IT_CC2)
	{
		u16 ccr2 = TIM4->CCR2;
		if(GPIOB->IDR & GPIO_Pin_7)
		{
			temp_cnt2 = ccr2;
			TIM4->CCER |= (1<<5);
		}
		else
		{
			temp_cnt2_2 = ccr2;
			TIM4->CCER &= ~(1<<5);
	//		ole[1]=PWM_IN_BUFFER[1];
			if(temp_cnt2_2>=temp_cnt2)
				PWM_IN_BUFFER[1] = temp_cnt2_2-temp_cnt2;
			else
				PWM_IN_BUFFER[1] = 2000-temp_cnt2+temp_cnt2_2;
//			if((PWM_IN_BUFFER[1]<1000)||(PWM_IN_BUFFER[1]>2000))PWM_IN_BUFFER[1]=ole[1];

		}
	}
	if(TIM4->SR & TIM_IT_CC3)
	{
		u16 ccr3 = TIM4->CCR3;
		if(GPIOB->IDR & GPIO_Pin_8)
		{
			temp_cnt3 = ccr3;
			TIM4->CCER |= (1<<9);
		}
		else
		{
			temp_cnt3_2 = ccr3;
			TIM4->CCER &= ~(1<<9);
	//		ole[2]=PWM_IN_BUFFER[2];
			if(temp_cnt3_2>=temp_cnt3)
				PWM_IN_BUFFER[2] = temp_cnt3_2-temp_cnt3;
			else
				PWM_IN_BUFFER[2] = 2000-temp_cnt3+temp_cnt3_2;
	//		if((PWM_IN_BUFFER[2]<1000)||(PWM_IN_BUFFER[2]>2000))PWM_IN_BUFFER[2]=ole[2];
		}
	}
	if(TIM4->SR & TIM_IT_CC4)
	{
		u16 ccr4 = TIM4->CCR4;
		if(GPIOB->IDR & GPIO_Pin_9)
		{
			temp_cnt4 = ccr4;
			TIM4->CCER |= (1<<13);
		}
		else
		{
			temp_cnt4_2 = ccr4;
			TIM4->CCER &= ~(1<<13);
	//		ole[3]=PWM_IN_BUFFER[3];
			if(temp_cnt4_2>=temp_cnt4)
				PWM_IN_BUFFER[3] = temp_cnt4_2-temp_cnt4;
			else
				PWM_IN_BUFFER[3] = 2000-temp_cnt4+temp_cnt4_2;
	//		if((PWM_IN_BUFFER[3]<1000)||(PWM_IN_BUFFER[3]>2000))PWM_IN_BUFFER[3]=ole[3];
		}
		
	}
}
