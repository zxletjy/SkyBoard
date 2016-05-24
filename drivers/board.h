
#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f10x.h"
#include "string.h"
#include "Math.h"


/***************LED GPIO定义******************/
#define RCC_LED1		RCC_APB2Periph_GPIOB
#define GPIO_LED1		GPIOB
#define LED1_Pin		GPIO_Pin_5
/*********************************************/
/***************I2C GPIO定义******************/
#define SKYBOARD_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_10
#define I2C_Pin_SDA		GPIO_Pin_11
#define SKYBOARD_RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
/***************SPI GPIO定义******************/
#define SKYBOARD_GPIO_SPI		GPIOA
#define SKYBOARD_GPIO_CE			GPIOA
#define SPI_Pin_CSN			GPIO_Pin_4
#define SPI_Pin_SCK			GPIO_Pin_5
#define SPI_Pin_MISO		GPIO_Pin_6
#define SPI_Pin_MOSI		GPIO_Pin_7
#define SPI_Pin_CE			GPIO_Pin_8
#define RCC_GPIO_SPI		RCC_APB2Periph_GPIOA
#define RCC_GPIO_CE			RCC_APB2Periph_GPIOA
/*********************************************/

/***************硬件中断优先级******************/
#define NVIC_UART_P	5
#define NVIC_UART_S	1
/***********************************************/

#include "Config.h"
#include "LED.h"
#include "I2C_soft.h"
#include "MPU6050.h"
#include "PWM.h"
#include "EEPROM.h"
#include "PWM_In.h"
#include "usart1.h"
void SkyBoard_Init(void);
void SysTick_IRQ(void);
uint32_t GetSysTime_us(void);


#endif /* __BOARD_H__ */

// 




