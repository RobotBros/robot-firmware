#ifndef __SYS_H
#define __SYS_H	
#include "stm32f10x.h"
#include "Public.h"
#include "Robot.h"
#include "Customer.h"
#include "Robot_Command.h"
#include "Robot_Fault.h"
#include "Robot_Flash.h"
#include "Robot_Fault.h"
#include "Robot_Gear.h"
#include "Robot_Head_LED.h"
#include "Robot_IPK_Fault.h"
#include "Robot_LED.h"
#include "Robot_LPF.h"
#include "Robot_Poweron.h"
#include "Robot_Sample.h"
#include "Robot_State.h"

#define SYS_TICK_1MS					8999 		// 72M/9000/8 = 1ms

#define	ON								1
#define	OFF								0

/*Bit operation define--------------------------------------------------------*/
#define	BitSet(REG,NUM)  ((REG) |= (1UL<<(NUM)))
#define	BitClr(REG,NUM)  ((REG) &= (~(1UL<<(NUM))))
#define	BitXor(REG,NUM)  ((REG) ^= (1UL<<(NUM)))
#define	BitVal(REG,NUM)  ((REG)&(1UL<<(NUM)))
	 
//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����

/*Function  define------------------------------------------------------------*/
#define Disable_IRQ()  __disable_irq() 						 //Disable All Interrupt
#define Enable_IRQ()   __enable_irq()  						 //Enable All Interrupt
//#define Enable_WDT()   			//Enable watchdog timer  
//#define Disable_WDT()   //Disable watchdog timer 
//#define Clear_WDT()   //Clear watchdog timer 
//#define Fault_WDT()   //write Err magic word to Reset MCU 
//#define SW_Reset()	//SoftWare to reset MCU

void System_Init(void);

#endif

