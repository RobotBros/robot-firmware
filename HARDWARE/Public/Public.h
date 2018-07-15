#ifndef PUBLIC_H
#define PUBLIC_H

#include "Sys.h"
#include "Customer.h"
#include "Robot.h"

/*Function  define------------------------------------------------------------*/

/*Function  Declaration-------------------------------------------------------*/
void Public_Init(void);
void NVIC_Configuration(void);
static void SysTick_Init(void);
void SysTick_Handler(void);
void Delay(u16 delay_times);

#endif
