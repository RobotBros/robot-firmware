#include "sys.h"

int main(void)
{	
	System_Init();

  while(1)
	{	
		/* Robot Main Function-----------------------------------------------------*/
		Robot_Main();	

		/* Customer Main Function--------------------------------------------------*/
		Customer_Main();

		/* Clear Watchdog--------------------------------------------------------*/
		//Clear_WDT();
	}
	return 0;
}

