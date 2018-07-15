#ifndef __ROBOT_GEAR_H
#define __ROBOT_GEAR_H	 
#include "sys.h"

/*define Gear TX frame--------------------------------------------*/
typedef struct
{
    u8   	Id; 		 			// Gear ID
    u8   	Command;			// Gear Action Command
    u8   	Param1H;    	 		// parameter1 high byte
    u8	Param1L;    			// parameter1 low byte
    u8   	Param2H;      		// parameter2 high byte
    u8   	Param2L;      		// parameter2 ow byte
    u8   	Check_Sum; 		// Check_sum
}tGear_TX_Frame;

//------------Steering gear command frame
//TX frame:0xFA+0xAF+ID+command+parm1H+parm1L+parm2H+parm2L+checksum+0xED		Lenth 10 bytes
//TX checksum =  ID+command+parm1H+parm1L+parm2H+parm2L							Lenth  1 byte
//RX frame:0xFA+0xAF+ID+status+parm1H+parm1L+parm2H+parm2L+checksum+0xED			Lenth 10 bytes
//TX checksum =  ID+status+parm1H+parm1L+parm2H+parm2L								Lenth  1 byte
//-------------------------------------
#define ACTION_IDEL					0x00	
#define ACTION_TIME_COMMAND		0x01  	//0xFA+0XAF+ID+0x01+angle+TIME+timeH+timeL+checksum+0xED  ACK:0xAA+ID  TIME(20ms/1)
#define LED_COMMAND				0x04	//0xFA+0xAF+ID+0x04+LEDstate +0+0+0+0+checksum+0xED ACK:0xAA+ID
#define RD_ANGLE_COMMAND			0x02	//0xFA+0xAF+ID+0x02+0+0+0+0+checksum+0xED		      ACK:0xFA+0xAF+ID+0xXX+tagH+tagL+relH+relL+checksum+0xED
#define MODIFY_ID_COMMAND			0xCD	//0xFA+0xAF+ID+0xCD+0+ID(1-240)+0+0+checksum+0xED   ACK:0xFA+0xAF+ID+0xAA(EE)+0+ID+0+0+checksum+0xED
#define ANGLE_ADJ_COMMAND			0xD2	//0xFA+0xAF+ID+0xD2+FDH+FDL+BWH+BWL+checksum+0xED   ACK:0xFA+0xAF+ID+0xAA(EE)+0+0+0+0+checksum+0xED
#define RD_ADJ_COMMAND			0xD4	//0xFA+0xAF+ID+0xD4+0+0+0+0+checksum+0xED   		ACK:0xFA+0xAF+ID+0xAA(EE)+FDH+FDL+BWH+BWL+checksum+0xED

/*Gear check sum define------------------------------------------*/
#define ID1_CHECK_START			2
#define ID1_CHECK_END				8
#define ID2_CHECK_START			12
#define ID2_CHECK_END				18
#define ID3_CHECK_START			22
#define ID3_CHECK_END				28
#define ID4_CHECK_START			32
#define ID4_CHECK_END				38
#define ID5_CHECK_START			42
#define ID5_CHECK_END				48
#define ID6_CHECK_START			52
#define ID6_CHECK_END				58
#define ID7_CHECK_START			62
#define ID7_CHECK_END				68
#define ID8_CHECK_START			72
#define ID8_CHECK_END				78
#define ID9_CHECK_START			82
#define ID9_CHECK_END				88
#define ID10_CHECK_START			92
#define ID10_CHECK_END				98
#define ID11_CHECK_START			102
#define ID11_CHECK_END				108
#define ID12_CHECK_START			112
#define ID12_CHECK_END				118
#define ID13_CHECK_START			122
#define ID13_CHECK_END				128
#define ID14_CHECK_START			132
#define ID14_CHECK_END				138
#define ID15_CHECK_START			142
#define ID15_CHECK_END				148
#define ID16_CHECK_START			152
#define ID16_CHECK_END				158

/*GEAR DIR ---GPIOA.1------------------------------------------*/
#define	GEAR_DIR_RX()		GPIOA->ODR |= (1<<1)     
#define	GEAR_DIR_TX()		GPIOA->ODR &= ~(1<<1) 

void Robot_Gear_Var_Init(void);
void Robot_Steering_Gear_Init(void);
void Robot_Uart2_Init(u32 bound);
void Robot_Usart2_DMA_Init(void);
u8 Robot_Gear_Check_sum(u8 *buf, u8 start_byte_no, u8 end_byte_no);
void Robot_Gear_DMA1_USART2_TX(void);
void Gear_Tx_Frame(void);
void Robot_Steering_Gear_Main(void);
void Robot_Steering_Gear_TimeT1ms(void);
void DMA1_Channel7_IRQHandler(void);

#endif

