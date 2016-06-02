/********************Ӳ����ʼ��*****************************/
#include "board.h"
#include "usb_lib.h"
// cycles per microsecond
static volatile uint32_t usTicks = 0;
// �δ�ʱ���������� ,49������
static volatile uint32_t sysTickUptime = 0;

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}

void SysTick_IRQ(void)
{
	sysTickUptime++;
	scheduler.cnt_1ms++;
	scheduler.cnt_2ms++;
	scheduler.cnt_5ms++;
	scheduler.cnt_10ms++;
	scheduler.cnt_20ms++;
}

uint32_t GetSysTime_us(void) 
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void SkyBoard_Init(void)
{
	//�ж����ȼ��������
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	//��TIM4�����ж����ȼ���������SysTickӵ��Ĭ�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	/* ����P[A|B|C|D|E]13Ϊ�ж�Դ */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//��ʼ��ϵͳ�δ�ʱ��
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	
	
	USB_Init();
	
	//��ʼ��ģ��I2C
	I2C_Soft_Init();
	//��ʼ����ʱ�����PWM,24KHz
	PWM_Out_Init(24000);
	//��ʼ��PWM�����
  PWM_In_Init();
	
	LED_Init();
	//����flash
	FLASH_Unlock();	
	//��ʼ������eeprom����
	EE_Init();	
	
	USART1_Init(115200);
	
	Fc_Init();
	
	//��������ʼ��
	//��ʼ��MPU6050��1Khz�����ʣ�42Hz��ͨ�˲�
	MPU6050_Init(1000,42);
	
	SPI1_Init();
	
	NRF_Init(MODEL_RX2,80);
}

