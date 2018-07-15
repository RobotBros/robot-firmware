/*******************************************************************************
 * FileName£ºRobot_Command.c
 * Function:  Robot Command function - SPI Slave, Host is ESP32
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
#include "Robot_Command.h"
#include "Robot_LED.h"
#include "stm32f10x_spi.h"


/*static variable define----------------------------------------------------------*/ 
static u16 Motor_Command_Enable = 0;
static u16 Speaker_Command_Enable = 0;
static u16 Tfcard_Command_Enable = 0;
static u16 Debug_Command_Enable = 0;			// Debug mode Robot Action ctrl by ESP32
static u16 *CommandReg_Addr[4];


/*******************************************************************************
 * Routine   : Robot_Command_Out
 * Function  : Register Read From Robot Base Driver - STM32
 ******************************************************************************/
u16 Robot_Command_Out(u8 regist)
{
	if(regist < 4)	return (*CommandReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine     : Robot_Command_Init
 * Function    : Robot Command Control Initialization
 ******************************************************************************/
void Robot_Command_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	CommandReg_Addr[0] = &Motor_Command_Enable;
	CommandReg_Addr[1] = &Speaker_Command_Enable;
	CommandReg_Addr[2] = &Tfcard_Command_Enable;
	CommandReg_Addr[3] = &Debug_Command_Enable;
	
	Motor_Command_Enable = 0;	
	Speaker_Command_Enable = 0;
	Tfcard_Command_Enable = 0;
	Debug_Command_Enable = 0;
	
	// SPI Init
	
  /* Enable GPIOB Clock */  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Enable SPI2 Clock */  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  /* SPI1 Init */
  SPI_Cmd(SPI2, DISABLE);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	
  SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;                         // Slave mode
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                    // 8 Bit data	
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                          // High when IDLE
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                         // Sample at 2nd edge
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                            // Soft NSS
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // No effect
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                   // MSB mode
  SPI_InitStructure.SPI_CRCPolynomial = 7;                             // No effect
  SPI_Init(SPI2, &SPI_InitStructure);
  SPI_Cmd(SPI2, ENABLE);
	
	// SPI NVIC SETUP   
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;      				// SPI2 Channel
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  	// Piority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;    			// Subpiority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 
	SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE); // Enable recv interrupt  
}

void SPI2_IRQHandler(void)
{
	u8 data = 0;
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)  // Check if recv interrupt
  { 
		LED_INV();
    SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);    // Clear interrupt bit
    data = SPI_I2S_ReceiveData(SPI2);          						// Read data from SPI1
	}
	if (data == 1) {
		data += 1;
	}
}

/*******************************************************************************
 * Routine     : Robot_Command_Motor_Enable
 * Function    : Robot Motor Enable
 ******************************************************************************/
void Robot_Command_Motor_Enable(u8 enable)
{
	if(enable)	{Motor_Command_Enable = 1; ROBOT_POWER_ON();}
	else		Motor_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_Speaker_Enable
 * Function    : Robot Speaker Enable
 ******************************************************************************/
void Robot_Command_Speaker_Enable(u8 enable)
{
	if(enable)	Speaker_Command_Enable = 1;
	else		Speaker_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_Debug_Enable
 * Function    : Robot Debug mode Enable
 ******************************************************************************/
void Robot_Command_Debug_Enable(u8 enable)
{
	if(enable)	Tfcard_Command_Enable = 1;
	else		Tfcard_Command_Enable = 0;
}

/*******************************************************************************
 * Routine     : Robot_Command_TFCARD_Enable
 * Function    : Robot TF Card Enable
 ******************************************************************************/
void Robot_Command_TFCARD_Enable(u8 enable)
{
	if(enable)	Debug_Command_Enable = 1;
	else		Debug_Command_Enable = 0;
}

 //END*************************************************************************/
 
