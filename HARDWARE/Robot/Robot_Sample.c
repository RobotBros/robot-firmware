/*******************************************************************************
 * FileName��Robot.c
 * Function: Robot base function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	  
#include "Robot_Sample.h"

/*static variable define----------------------------------------------------------*/ 		
static u8 T10msFlag_Sampling = 0;
static u16 SampReg_Original[2];				// Sample register witch no filter 0:Vbat 1:Ibus
static u16 *SampReg_Addr[2];					// Sample register witch filter 0:Vbat 1:Ibus
static u16 Vbat_val = 0;						// Smaple with no filter
static u16 Ibus_val = 0;
static u16 Vbat_val_f = 0;						// Smaple with filter
static u16 Ibus_val_f = 0;
static tFilt VbatFilt = {0,0,0,0,0,0,0,0};
static tFilt IbusFilt = {0,0,0,0,0,0,0,0};

/*******************************************************************************
 * Routine     : Robot_Sample_Init
 * Function    : Robot sample init
 ******************************************************************************/
void Robot_Sample_Init(void)
{
/*variable initial----------------------------------------------------------------*/ 	
	SampReg_Addr[ADC1_VBAT_BYTE] = (u16*)&Vbat_val_f;
	SampReg_Addr[ADC1_IBUS_BYTE] = (u16*)&Ibus_val_f;
	SampReg_Original[ADC1_VBAT_BYTE] = Vbat_val;
	SampReg_Original[ADC1_IBUS_BYTE] = Ibus_val;

	VbatFilt.Enable = 1;
	VbatFilt.N= 2;							
	IbusFilt.Enable = 1;
	IbusFilt.N = 2;							
/*ADC initial-------------------------------------------------------------------*/ 
	Robot_Adc_Init();
}

/*******************************************************************************
 * Routine     : Robot_Adc_Init
 * Function    : Robot ADC Initial
 ******************************************************************************/		   
void  Robot_Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
/*Initial ADC DMA Funciton------------------------------------------------*/
	Robot_Adc_DMA_Init();				
/*Initial ADC Sampling Clock----------------------------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	| RCC_APB2Periph_AFIO, ENABLE );	  
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   																// Clk = 72M/6=12MHz
/*Initial ADC IO Initial PC0(IN10) PC1(IN11)---------------------------------*/                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
/*Initial ADC Reg-------------------------------------------------------*/
	ADC_DeInit(ADC1);  												// Reset ADC1 register
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;				//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			  			//���ŵ�ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;					//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;				//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 2;								//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);								//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
/*Initial ADC Rank in the regular group sequencer----------------------------*/
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5 );     // Sample time 239.5 cycles           
        ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5 );
/*Enable ADC and DMA--------------------------------------------------*/
  	ADC_DMACmd(ADC1, ENABLE);       									//ʹ��ADC1��DMA����      
	ADC_Cmd(ADC1, ENABLE);											//ʹ��ָ����ADC1	
	ADC_ResetCalibration(ADC1);										//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));						//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);								  		//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));		 					 //�ȴ�У׼����

/*DMA1 Channel1 Interrupt------------------------------------------------------------*/
	NVIC_SetPriority(DMA1_Channel1_IRQn,1);							// IRQ1
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);	
  	//NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;								
	//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;									
	//NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;									
	//NVIC_Init(&NVIC_InitStructure);			
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);				 													 
/*DMA and ADC1 enable---------------------------------------------------------------*/	
	DMA_Cmd(DMA1_Channel1, ENABLE);     								 //����DMAͨ��
    	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	 					 //�������ADת��
}	

/*******************************************************************************
 * Routine     : Robot_Adc_DMA_Init
 * Function    : Robot Adc DMA Initial
 ******************************************************************************/	
void Robot_Adc_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure; 

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);						// ʹ��DMA1ʱ��	DMA_DeInit(DMA1_Channel1);												// ��λDMA1 ͨ��1

	DMA_DeInit(DMA1_Channel1);  												 //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1_DR_Address;				// ����ADC1��ַ ADC1->DR
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&SampReg_Original;  			// DMA1�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  						// ���ݴ��䷽�򣬴������ȡ���͵��ڴ�
	DMA_InitStructure.DMA_BufferSize = 2;  										// DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 				// �����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					// �ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  	// ADC���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 		// �������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  							// ������ѭ������ģʽ��������ɺ��ִӳ�ʼ��λ��ִ��
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 						// DMAͨ�� xӵ�и����ȼ� 0
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 								// DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  	
}

/*******************************************************************************
 * Routine   : Robot_Sample_Out
 * Function  : Read ADC Value 
 * Remark   : Return ROBOT ADC Value:regist:ADC1_VBAT,ADC1_IBUS....
 ******************************************************************************/
u16 Robot_Sample_Out(u8 regist, u8 fliter_en)
{  
	if(fliter_en)
	{
		if(regist < 2)	return (*SampReg_Addr[regist]);
		else			return(0);
	}
	else
	{
		if(regist < 2)	return (SampReg_Original[regist]);
		else			return(0);
	}
}

/*******************************************************************************
 * Routine     : Robot_Sample_Main
 * Function    : Robot sample Main loop
 ******************************************************************************/
 void Robot_Sample_Main(void)
{
	//uint16 V_tmp = 0;
	//uint16 I_tmp= 0;
	
	if(T10msFlag_Sampling)
	{
		T10msFlag_Sampling = 0;
	 	//V_tmp = Robot_Sample_Out(ADC1_VBAT_BYTE, FILITER_ENABLE);
	 	//I_tmp  = Robot_Sample_Out(ADC1_IBUS_BYTE, FILITER_ENABLE);
	}
}

/*******************************************************************************
 * Routine     : Robot_Sample_Timer1ms
 * Function    : 1ms interrupt
 ******************************************************************************/
void Robot_Sample_TimeT1ms(void)
{
	static u8 T1msCnt=0;
	static u8 T10msCnt=0;
	
	/*10ms-------------------------------------------------------------------*/
	if(++T1msCnt>=10)
	{
		T1msCnt = 0;
		T10msCnt++;
		
		T10msFlag_Sampling = 1;
	}
}

/*******************************************************************************
 * Routine     : DMA1_Channel1_IRQHandler
 * Function    : DMA1 CH1-ADC1 IRQ Handle
 ******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC1))
	{
		Vbat_val_f =  (u16)LPFilt_2N(SampReg_Original[ADC1_VBAT_BYTE], &VbatFilt);
		Ibus_val_f =  (u16)LPFilt_2N(SampReg_Original[ADC1_IBUS_BYTE], &IbusFilt);
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
}

