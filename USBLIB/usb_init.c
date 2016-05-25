/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : usb_init.c
* Author             : MCD Application Team
* Version            : V2.2.0
* Date               : 06/13/2008
* Description        : Initialization routines & global variables
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "hw_config.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*  The number of current endpoint, it will be used to specify an endpoint */
 u8	EPindex;
/*  The number of current device, it is an index to the Device_Table */
/* u8	Device_no; */
/*  Points to the DEVICE_INFO structure of current device */
/*  The purpose of this register is to speed up the execution */
DEVICE_INFO *pInformation;
/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
DEVICE_PROP *pProperty;
/*  Temporary save the state of Rx & Tx status. */
/*  Whenever the Rx or Tx state is changed, its value is saved */
/*  in this variable first and will be set to the EPRB or EPRA */
/*  at the end of interrupt process */
u16	SaveState ;
u16  wInterrupt_Mask;
DEVICE_INFO	Device_Info;
USER_STANDARD_REQUESTS  *pUser_Standard_Requests;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

//初始化USB
void USB_Init(void)
{
	Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
	pInformation = &Device_Info;
	pInformation->ControlState = 2;
	pProperty = &Device_Property;
	pUser_Standard_Requests = &User_Standard_Requests;
	/* Initialize devices one by one */
	pProperty->Init();
}
#include "usb_pwr.h" 
#include "usb_desc.h"
#include "usb_regs.h"
#include <string.h>
//检查USB是否连接上
bool USB_Check_Conn(void)
{
	static u8 fnr = 0;
	bool state = FALSE;
	u8 a = GetFNR();
	if(bDeviceState == CONFIGURED)
	{
		if (fnr != a)
		{
			fnr = a;
			state = TRUE;
		}
	}
	else
	{
		state = FALSE;
	}
	return state;
}
extern u8 USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
extern u32 USB_Rx_Count;		//USB接收到几个数据
bool USB_Virtual_Serial_Send(uint8_t *p, uint8_t length)
{
	bool err = TRUE;
	
	if (USB_Check_Conn() == TRUE)
	{
		UserToPMABufferCopy(p, ENDP1_TXADDR, length);
		SetEPTxCount(ENDP1, length);
		SetEPTxValid(ENDP1);
	}
	else
	{
		err = FALSE;
	}
	return err;
}
bool USB_Virtual_Serial_Receive(uint8_t *p, uint8_t *length)
{
	bool err = TRUE;
	if ((USB_Rx_Count != 0) && (USB_Check_Conn() == TRUE))
  {
		memcpy(p, USB_Rx_Buffer, USB_Rx_Count);
		memcpy(length, &USB_Rx_Count, 1);
		USB_Rx_Count = 0;
  }
	else
	{
		err = FALSE;
	}
	return err;
}
#include "usb_istr.h"
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  USB_Istr();
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
