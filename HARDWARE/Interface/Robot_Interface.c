/*******************************************************************************
 * FileName: SDHET.c
 * Function: SDHET interface Function
 * Designer: Sunlufeng/2017.07.24
 * MCU: Infineon XMC1400 series
*******************************************************************************/
#include "SDHET.h"
/*******************************************************************************
 * Routine   : Customer_Init
 * Function  : Customer Initialization Function
 * Remark    : Do not change the sequence of Inverter_Init and Driver_Init
 ******************************************************************************/
void SDHET_Init(void)
{
	Inverter_Init(); //Inverter_Init must before Driver_Init Initialization

	Driver_Init();  //Driver_Init must after Inverter_Init Initialization
}
/*******************************************************************************
 * Routine   : Customer_Main
 * Function  : Customer Main loop Function
 * Remark    : 
 ******************************************************************************/
void SDHET_Main(void)
{
	Inverter_Main();

	Driver_Main();
}
/*******************************************************************************
 * Routine     : Receive_Data
 * Function    : 
 ******************************************************************************/
uint8 Receive_Data(void)
{
	uint8 temp=0;

	temp=USIC0_CH0->RBUF;
	return(temp);
}
/*******************************************************************************
 * Routine     : Send_Data
 * Function    : 
 ******************************************************************************/
void Send_Data(uint8 temp)
{
	 USIC0_CH0->TBUF[0]=temp;
}