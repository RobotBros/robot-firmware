#ifndef __ROBOT_POWERON_H
#define __ROBOT_POWERON_H 

#include "sys.h"
#include "Robot_State.h"
#include "Robot_Fault.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x.h"

//GPIOC.15
#define	ROBOT_POWER_ON()		GPIOC->ODR |= (1<<15)     
#define	ROBOT_POWER_OFF()		GPIOC->ODR &= ~(1<<15) 


void Robot_Poweron_Init(void);
	 				    
#endif
