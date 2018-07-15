/*******************************************************************************
 * FileName£ºRobot_Poweron.c
 * Function: Robot_Power on  function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_Poweron.h"
	
/*******************************************************************************
 * Routine: Robot_Poweron_Init
 * Function: Poweron Port Initialization
 ******************************************************************************/
void Robot_Poweron_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);			// Enable GPIOB clk
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;							// PC15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					// Push pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					//IO speed is 50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);								// Write GPIO config
	ROBOT_POWER_OFF(); 											// Power on 
}

