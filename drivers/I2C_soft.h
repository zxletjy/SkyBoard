#ifndef __SKYBOARD_DRV_I2C_SOFT2_H__
#define __SKYBOARD_DRV_I2C_SOFT2_H__

#include "stm32f10x.h"


#define SCL_H         SKYBOARD_GPIO_I2C->BSRR = I2C_Pin_SCL
#define SCL_L         SKYBOARD_GPIO_I2C->BRR  = I2C_Pin_SCL
#define SDA_H         SKYBOARD_GPIO_I2C->BSRR = I2C_Pin_SDA
#define SDA_L         SKYBOARD_GPIO_I2C->BRR  = I2C_Pin_SDA
#define SCL_read      SKYBOARD_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      SKYBOARD_GPIO_I2C->IDR  & I2C_Pin_SDA

void I2C_Soft_Init(void);
int I2C_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);	
int I2C_Single_Read(u8 SlaveAddress,u8 REG_Address);
int I2C_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

#endif

