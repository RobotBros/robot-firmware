/*******************************************************************************
 * FileName£ºRobot_IPK_Fault.c
 * Function: Robot Input Current Peak Fault Handle function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_IPK_Fault.h"

/*static variable define-----------------------------------------------------------*/ 
static u8 IPK_state = 0;

/******************************************************************************
 * Routine     : Robot_IPK_Fault_Init
 * Function    : Robot IPK Fault I/O Initialization GPIOC.14
 ******************************************************************************/
void Robot_IPK_Fault_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);											
	
/*GPIOC.14 IPK Fault I/O-----------------------------------------------------*/		
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource14);

  	EXTI_InitStructure.EXTI_Line=EXTI_Line14;																		
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);																					  	

	NVIC_SetPriority(EXTI15_10_IRQn,0);							//IRQ0										
	//NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	//NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;														
  	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;								
  	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;												
  	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;															
  	//NVIC_Init(&NVIC_InitStructure);  	  		

	IPK_state = 0;
}

/*******************************************************************************
 * Routine   : Robot_Hardware_Fault_Out
 * Function  : Register Read From Robot IPK Fault
 ******************************************************************************/
u8 Robot_Hardware_Fault_Out(void)
{  
	return IPK_state;
}

/******************************************************************************
 * Routine     : EXTI15_10_IRQHandler
 * Function    : Robot IPK Fault Interrupt Handle
 ******************************************************************************/
void EXTI15_10_IRQHandler(void)
{	
	//u8 static cnt8 = 0;
	
	if(EXTI_GetFlagStatus(EXTI_Line14) != RESET)
	{
		//cnt8++;
		//Delay(10);
		//if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_15) == 0)
		//{
			IPK_state = 1;
			ROBOT_POWER_OFF();						// Turn OFF Gear Power
		//}
		//else
		//{
		//	IPK_state = 0;
		//}
		EXTI_ClearITPendingBit(EXTI_Line14);  							 						
	}
}

