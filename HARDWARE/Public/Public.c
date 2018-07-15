/*******************************************************************************
 * FileName£ºPublic.c
 * Function: Public function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Public.h"

/*******************************************************************************
 * Routine   : Public_Init
 * Function  : Public General Initialization
 ******************************************************************************/
void Public_Init(void)
{
	/*Interrupt Node set ---------------------------------------------------*/
	NVIC_Configuration(); 
	
	/* Watchdog Initialization -----------------------------------------------*/
	//WTD_Init();
  
	/* Timer Interrupt Initialization of Public  Time base--------------------------*/
    	SysTick_Init();	
}

/*******************************************************************************
 * Routine   : NVIC_Configuration
 * Function  : Interrupt Node Assignment
 ******************************************************************************/
void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);	//4 //4 bits for pre-emption priority, 0 bits for subpriority
}

/*******************************************************************************
 * Routine: SysTick_Init
 * Function: Timer Interrupt Initialization(Core SysTick Timer)
 ******************************************************************************/
static void SysTick_Init(void)
{
	SysTick_Config(SYS_TICK_1MS);							// 1MS
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	// DIV 8										// Systick = HCLK/8
	//SysTick->LOAD = SYS_TICK_1MS; 																	// System timer for 1ms
	//SysTick->CTRL|= SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk |SysTick_CTRL_CLKSOURCE_Msk;   	// Enable systick and IRQ

    	//NVIC_SetPriority(SysTick_IRQn, 1);
}

/*******************************************************************************
 * Routine : delay
 * Function: delay times
 ******************************************************************************/
void Delay(u16 delay_times)
{
	u16 temp16;
	
	temp16 = delay_times;
	//Clear_WDT();
	while(temp16--);
}

/*******************************************************************************
 * Routine   : WTD_Init
 * Function  : WatchDog Initialization
 * Remark    : WatchDog Clock is fOFI(24MHz)
 ******************************************************************************/
/*static void WTD_Init(void)
{
	
}*/

/*******************************************************************************
 * Routine   : HardFault_Handler
 * Function  : HardFault Handler interruppt
 ******************************************************************************/
/*void HardFault_Handler(void)
{
	SW_Reset(); // If hardware ERR to reset MCU by Software
}*/

/*******************************************************************************
 * Routine: SysTick_Handler
 * Function: SysTick Timer interruppt Handler Routine(1ms interruppt)
 ******************************************************************************/
void SysTick_Handler(void)
{
    	Robot_Action_TMR_1MS(); 
    	Customer_TMR_1MS();
/*TEST time base--------------------------------------------------------------*/		
	//LED_INV();
}

