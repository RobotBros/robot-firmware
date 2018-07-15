/*******************************************************************************
 * FileName：Robot_Gear.c
 * Function: Robot Gear drive function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	    
#include "Robot_Gear.h"

/*Array Size define-------------------------------------------------------------*/
#define GEAR_TXD_SZIE     	160 			// Gear1 to Gear16 Command Action = 10*16 = 160 Bytes
#define GEAR_RXD_SZIE     	10
/*static variable define----------------------------------------------------------*/ 
static u8 Gear_tx_buf[GEAR_TXD_SZIE] = {0}; 
static u8 T500msFlag_Gear_Action = 0;		// TX time gap fLag
static u8 Gear_TX_Frame_Done_Flag = 1;	// TX is done flag
/*static tGear_TX_Frame  Gear1,Gear2,Gear3,Gear4,Gear5,Gear6,Gear7,Gear8,
					Gear9,Gear10,Gear11,Gear12,Gear13,Gear14,Gear15,
					Gear16;
*/

/*******************************************************************************
 * Routine     : Robot_Gear_Var_Init
 * Function    : Robot  Gear Variable Initialization 
 ******************************************************************************/
void Robot_Gear_Var_Init(void)
{

//Left hand side----------------------------------------------------------------//
//Gear1 variable define----------------------------------------------------------// 
	Gear_tx_buf[0] = 0xFA;
	Gear_tx_buf[1] = 0xAF;
	Gear_tx_buf[2] = 1;							// ID 1
	Gear_tx_buf[3] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[4] = 90;							// Angle
	Gear_tx_buf[5] = 50;							// Action time
	Gear_tx_buf[6] = 0;
	Gear_tx_buf[7] = 0;
	Gear_tx_buf[8] = Robot_Gear_Check_sum(Gear_tx_buf,ID1_CHECK_START,ID1_CHECK_END);
	Gear_tx_buf[9] = 0xED;						//ED
	
//Gear2 variable define----------------------------------------------------------// 	
	Gear_tx_buf[10] = 0xFA;
	Gear_tx_buf[11] = 0xAF;
	Gear_tx_buf[12] = 2;							// ID 2
	Gear_tx_buf[13] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[14] = 0;							// Angle
	Gear_tx_buf[15] = 50;						// Action time
	Gear_tx_buf[16] = 0;
	Gear_tx_buf[17] = 0;
	Gear_tx_buf[18] = Robot_Gear_Check_sum(Gear_tx_buf,ID2_CHECK_START,ID2_CHECK_END);
	Gear_tx_buf[19] = 0xED;						//ED

//Gear3 variable define----------------------------------------------------------// 	
	Gear_tx_buf[20] = 0xFA;
	Gear_tx_buf[21] = 0xAF;
	Gear_tx_buf[22] = 3;							// ID 3
	Gear_tx_buf[23] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[24] = 90;						// Angle
	Gear_tx_buf[25] = 50;						// Action time
	Gear_tx_buf[26] = 0;
	Gear_tx_buf[27] = 0;
	Gear_tx_buf[28] = Robot_Gear_Check_sum(Gear_tx_buf,ID3_CHECK_START,ID3_CHECK_END);
	Gear_tx_buf[29] = 0xED;						//ED
	
//Right hand side----------------------------------------------------------------//
//Gear4 variable define----------------------------------------------------------// 	
	Gear_tx_buf[30] = 0xFA;
	Gear_tx_buf[31] = 0xAF;
	Gear_tx_buf[32] = 4;							// ID 4
	Gear_tx_buf[33] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[34] = 90;						// Angle
	Gear_tx_buf[35] = 50;						// Action time
	Gear_tx_buf[36] = 0;
	Gear_tx_buf[37] = 0;
	Gear_tx_buf[38] = Robot_Gear_Check_sum(Gear_tx_buf,ID4_CHECK_START,ID4_CHECK_END);
	Gear_tx_buf[39] = 0xED;						//ED
//Gear5 variable define----------------------------------------------------------// 	
	Gear_tx_buf[40] = 0xFA;
	Gear_tx_buf[41] = 0xAF;
	Gear_tx_buf[42] = 5;							// ID 5
	Gear_tx_buf[43] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[44] = 180;							// Angle
	Gear_tx_buf[45] = 50;						// Action time
	Gear_tx_buf[46] = 0;
	Gear_tx_buf[47] = 0;
	Gear_tx_buf[48] = Robot_Gear_Check_sum(Gear_tx_buf,ID5_CHECK_START,ID5_CHECK_END);
	Gear_tx_buf[49] = 0xED;						//ED
//Gear6 variable define----------------------------------------------------------// 	
	Gear_tx_buf[50] = 0xFA;
	Gear_tx_buf[51] = 0xAF;
	Gear_tx_buf[52] = 6;							// ID 6
	Gear_tx_buf[53] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[54] = 90;						// Angle
	Gear_tx_buf[55] = 50;						// Action time
	Gear_tx_buf[56] = 0;
	Gear_tx_buf[57] = 0;
	Gear_tx_buf[58] = Robot_Gear_Check_sum(Gear_tx_buf,ID6_CHECK_START,ID6_CHECK_END);
	Gear_tx_buf[59] = 0xED;						//ED
	
//Left leg side----------------------------------------------------------------//
//Gear7 variable define----------------------------------------------------------// 	
	Gear_tx_buf[60] = 0xFA;
	Gear_tx_buf[61] = 0xAF;
	Gear_tx_buf[62] = 7;							// ID 7
	Gear_tx_buf[63] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[64] = 0;							// Angle
	Gear_tx_buf[65] = 50;						// Action time
	Gear_tx_buf[66] = 0;
	Gear_tx_buf[67] = 0;
	Gear_tx_buf[68] = Robot_Gear_Check_sum(Gear_tx_buf,ID7_CHECK_START,ID7_CHECK_END);
	Gear_tx_buf[69] = 0xED;						//ED

//Gear8 variable define----------------------------------------------------------// 	
	Gear_tx_buf[70] = 0xFA;
	Gear_tx_buf[71] = 0xAF;
	Gear_tx_buf[72] = 8;							// ID 8
	Gear_tx_buf[73] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[74] = 90;						// Angle
	Gear_tx_buf[75] = 50;						// Action time
	Gear_tx_buf[76] = 0;
	Gear_tx_buf[77] = 0;
	Gear_tx_buf[78] = Robot_Gear_Check_sum(Gear_tx_buf,ID8_CHECK_START,ID8_CHECK_END);
	Gear_tx_buf[79] = 0xED;						//ED	
//Gear9 variable define----------------------------------------------------------// 	
	Gear_tx_buf[80] = 0xFA;
	Gear_tx_buf[81] = 0xAF;
	Gear_tx_buf[82] = 9;							// ID 9
	Gear_tx_buf[83] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[84] = 90;						// Angle
	Gear_tx_buf[85] = 50;						// Action time
	Gear_tx_buf[86] = 0;
	Gear_tx_buf[87] = 0;
	Gear_tx_buf[88] = Robot_Gear_Check_sum(Gear_tx_buf,ID9_CHECK_START,ID9_CHECK_END);
	Gear_tx_buf[89] = 0xED;						//ED
//Gear10 variable define----------------------------------------------------------// 	
	Gear_tx_buf[90] = 0xFA;
	Gear_tx_buf[91] = 0xAF;
	Gear_tx_buf[92] = 10;						// ID 10
	Gear_tx_buf[93] = ACTION_TIME_COMMAND;		// Action Command
	Gear_tx_buf[94] = 90;						// Angle
	Gear_tx_buf[95] = 50;						// Action time
	Gear_tx_buf[96] = 0;
	Gear_tx_buf[97] = 0;
	Gear_tx_buf[98] = Robot_Gear_Check_sum(Gear_tx_buf,ID10_CHECK_START,ID10_CHECK_END);
	Gear_tx_buf[99] = 0xED;						//ED
//Gear11 variable define----------------------------------------------------------// 	
	Gear_tx_buf[100] = 0xFA;
	Gear_tx_buf[101] = 0xAF;
	Gear_tx_buf[102] = 11;						// ID 11
	Gear_tx_buf[103] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[104] = 90;						// Angle
	Gear_tx_buf[105] = 50;						// Action time
	Gear_tx_buf[106] = 0;
	Gear_tx_buf[107] = 0;
	Gear_tx_buf[108] = Robot_Gear_Check_sum(Gear_tx_buf,ID11_CHECK_START,ID11_CHECK_END);
	Gear_tx_buf[109] = 0xED;						//ED

//Right leg side----------------------------------------------------------------//
//Gear12 variable define----------------------------------------------------------// 	
	Gear_tx_buf[110] = 0xFA;
	Gear_tx_buf[111] = 0xAF;
	Gear_tx_buf[112] = 12;						// ID 12
	Gear_tx_buf[113] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[114] = 0;						// Angle
	Gear_tx_buf[115] = 50;						// Action time
	Gear_tx_buf[116] = 0;
	Gear_tx_buf[117] = 0;
	Gear_tx_buf[118] = Robot_Gear_Check_sum(Gear_tx_buf,ID12_CHECK_START,ID12_CHECK_END);
	Gear_tx_buf[119] = 0xED;						//ED
//Gear13 variable define----------------------------------------------------------// 	
	Gear_tx_buf[120] = 0xFA;
	Gear_tx_buf[121] = 0xAF;
	Gear_tx_buf[122] = 13;						// ID 13
	Gear_tx_buf[123] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[124] = 90;						// Angle
	Gear_tx_buf[125] = 50;						// Action time
	Gear_tx_buf[126] = 0;
	Gear_tx_buf[127] = 0;
	Gear_tx_buf[128] = Robot_Gear_Check_sum(Gear_tx_buf,ID13_CHECK_START,ID13_CHECK_END);
	Gear_tx_buf[129] = 0xED;						//ED
//Gear14 variable define----------------------------------------------------------// 	
	Gear_tx_buf[130] = 0xFA;
	Gear_tx_buf[131] = 0xAF;
	Gear_tx_buf[132] = 14;						// ID 14
	Gear_tx_buf[133] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[134] = 90;						// Angle
	Gear_tx_buf[135] = 50;						// Action time
	Gear_tx_buf[136] = 0;
	Gear_tx_buf[137] = 0;
	Gear_tx_buf[138] = Robot_Gear_Check_sum(Gear_tx_buf,ID14_CHECK_START,ID14_CHECK_END);
	Gear_tx_buf[139] = 0xED;						//ED
//Gear15 variable define----------------------------------------------------------// 	
	Gear_tx_buf[140] = 0xFA;
	Gear_tx_buf[141] = 0xAF;
	Gear_tx_buf[142] = 15;						// ID 15
	Gear_tx_buf[143] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[144] = 90;						// Angle
	Gear_tx_buf[145] = 50;						// Action time
	Gear_tx_buf[146] = 0;
	Gear_tx_buf[147] = 0;
	Gear_tx_buf[148] = Robot_Gear_Check_sum(Gear_tx_buf,ID15_CHECK_START,ID15_CHECK_END);
	Gear_tx_buf[149] = 0xED;						//ED
//Gear16 variable define----------------------------------------------------------// 	
	Gear_tx_buf[150] = 0xFA;
	Gear_tx_buf[151] = 0xAF;
	Gear_tx_buf[152] = 16;						// ID 16
	Gear_tx_buf[153] = ACTION_TIME_COMMAND;	// Action Command
	Gear_tx_buf[154] = 84;						// Angle
	Gear_tx_buf[155] = 50;						// Action time
	Gear_tx_buf[156] = 0;
	Gear_tx_buf[157] = 0;
	Gear_tx_buf[158] = Robot_Gear_Check_sum(Gear_tx_buf,ID16_CHECK_START,ID16_CHECK_END);
	Gear_tx_buf[159] = 0xED;						//ED

}

/******************************************************************************
 * Routine     : Robot_Steering_Gear_Init
 * Function    : Usart2 For Steering Gear Initialization
 ******************************************************************************/
void Robot_Steering_Gear_Init(void)
{
/*Usart2 DIR I/O-----------------------------------------------------*/
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);			// Enable GPIOA clk
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1 ;							// PA1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 					// Push pull mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					// IO speed is 50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);								// Write GPIO config
	GEAR_DIR_TX();													// Gear DIR TX mode
 /*Usart2 Initialization-------------------------------------------------*/
	Robot_Uart2_Init(115200);	 									// Gear Lower left interface
 /*Gear TX buffer Initialization-------------------------------------------------*/
	Robot_Gear_Var_Init();
}

/******************************************************************************
 * Routine     : Robot_Uart2_Init
 * Function    : Usart2 For Steering Gear Initialization
 ******************************************************************************/
void Robot_Uart2_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
/*Initial USART2 DMA Funciton----------------------------------------------------*/
	Robot_Usart2_DMA_Init();
/*Initial USART Clock-----------------------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);							// Enable GPIOA  				
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);							// Enable Usart2 clk
	USART_DeInit(USART2); 															// Reset Usart2 register
	
/*USART2 I/O TX GPIOA.2-----------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;															
  	GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
/*USART2 I/O RX GPIOA.3-----------------------------------------------------*/ 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;								
  	GPIO_Init(GPIOA, &GPIO_InitStructure);  

/*USART2 Initialization---------------------------------------------------------*/
	USART_InitStructure.USART_BaudRate = bound;										
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						// Data length is 8 bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								// One stop bit
	USART_InitStructure.USART_Parity = USART_Parity_No;									// No check bit
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		// No hardware ctrl bit
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					// RX and TX mode
  	USART_Init(USART2, &USART_InitStructure); 											
																																//如果使能了接收  
/*USART2 Interrupt------------------------------------------------------------*/
  	//NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;								
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;									
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									
	//NVIC_Init(&NVIC_InitStructure);														
	//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);				 
	//USART_ITConfig(USART2, USART_IT_TXE,   ENABLE);				
  	               
/*Enable USART2 and DMA--------------------------------------------------*/
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE); 				//使能USART2的 TX DMA传输    
     	USART_Cmd(USART2, ENABLE); 								   	

/*DMA1 Channel1 Interrupt------------------------------------------------------------*/
	NVIC_SetPriority(DMA1_Channel7_IRQn,3);							// DMA1 CH7--- IRQ3
	NVIC_EnableIRQ(DMA1_Channel7_IRQn);								// Enable DMA1 CH7
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);					// Transfer finish interrupt		 													 
}

/******************************************************************************
 * Routine     : Robot_Usart2_DMA_Init
 * Function    : Usart2 DMA TX/RX initial
 ******************************************************************************/
void Robot_Usart2_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						//DMA1
	
    	DMA_DeInit(DMA1_Channel7);  												//DMA1 CH7 --TX
	//DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);  				//DMA1 USART2 --TX
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&Gear_tx_buf;  				//DMA1内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  						//数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = GEAR_TXD_SZIE;  							//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  				//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 			//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  							//工作在正常缓存模式 single TX
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有中优先级 1
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  								//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);  																										
}

/******************************************************************************
 * Routine     : Robot_Gear_Check_sum
 * Function    : Gear frame checksum
 * check sum = ID+Command+par1+par2+par3+par4
 * buf[2]~buf[7]
 ******************************************************************************/
u8 Robot_Gear_Check_sum(u8 *buf, u8 start_byte_no, u8 end_byte_no)		
{
	u8 check_sum = 0;
	u8 cnt = 0;

	for(cnt=start_byte_no;cnt<end_byte_no;cnt++)
		{check_sum += buf[cnt];}
	return check_sum;
}

/******************************************************************************
 * Routine     : Robot_Gear_DMA1_TX
 * Function    : Start one times for Gear DMA1 TX buff-160 Bytes
 ******************************************************************************/
void Robot_Gear_DMA1_USART2_TX(void)
{ 
	DMA_Cmd(DMA1_Channel7, DISABLE );  						//关闭USART2 TX DMA1 CH7 所指示的通道      
 	DMA_SetCurrDataCounter(DMA1_Channel7,GEAR_TXD_SZIE);	//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA1_Channel7, ENABLE);  						//使能USART2 TX DMA1 CH7 所指示的通道 
}

/******************************************************************************
 * Routine     : Gear_Tx_Frame
 * Function    : Gear Tx frame buffer
 *TX frame:0xFA+0xAF+ID+command+parm1H+parm1L+parm2H+parm2L+checksum+0xED		Lenth 10 bytes
 *TX checksum =  ID+command+parm1H+parm1L+parm2H+parm2L							Lenth  1 byte
 *RX frame:0xFA+0xAF+ID+status+parm1H+parm1L+parm2H+parm2L+checksum+0xED			Lenth 10 bytes
 *TX checksum =  ID+status+parm1H+parm1L+parm2H+parm2L								Lenth  1 byte
 ******************************************************************************/
void Gear_Tx_Frame(void)
{
	GEAR_DIR_TX();							// TX mode
/*Gear tx frame--------------------------------------------------------*/
	Robot_Gear_DMA1_USART2_TX();			// Start Usart2 TX DMA Transfer
}

/*******************************************************************************
 * Routine     : Robot_Steering_Gear_Main
 * Function    : Robot Steering Gear Main Loop 
 ******************************************************************************/
void Robot_Steering_Gear_Main(void)
{	
	u8 state;
	
	if(T500msFlag_Gear_Action)
	{
		T500msFlag_Gear_Action = 0;
		
		if(Gear_TX_Frame_Done_Flag)					// TX DMA Transfer is done
		{
			state = Robot_State_Out(ROBOT_SystemState);
			if(state == ROBOT_STATE_RUN)
			{
				Gear_TX_Frame_Done_Flag = 0;
				Gear_Tx_Frame();						// Action
			}
			else if(state == ROBOT_STATE_DEBUG)
			{
				Gear_TX_Frame_Done_Flag = 0;
				// Debug mode action
			}
		}
	}
}

/*******************************************************************************
 * Routine     : Robot_Steering_Gear_TimeT1ms
 * Function    : Robot Steering Gear timebase(200ms)
 ******************************************************************************/
void Robot_Steering_Gear_TimeT1ms(void)
{
    static u16 T1msCnt=0;

/*500ms----------------------------------------------------------------------*/
	if(++T1msCnt>=500)
	{
		T1msCnt = 0;
		T500msFlag_Gear_Action = 1;
	}
}

/*******************************************************************************
 * Routine     : DMA1_Channel7_IRQHandler
 * Function    : DMA1 CH7-USART2 TX IRQ Handle
 ******************************************************************************/
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7))
	{
		Gear_TX_Frame_Done_Flag = 1;				// TX DMA Transfer is Done
		GEAR_DIR_RX();								// RX mode
		DMA_ClearITPendingBit(DMA1_FLAG_TC7);
	}
}

