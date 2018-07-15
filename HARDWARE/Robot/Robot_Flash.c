/*******************************************************************************
 * FileName��Robot_Flash.c
 * Function:   Robot spi Flash read and write
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	

#include "Robot_Flash.h"

/*Array Size define-----------------------------------------------------------*/
#define ROBOT_ACTION_BYE_LEN    (1000) 				// Flash opration length
/*static variable define----------------------------------------------------------*/ 
static u16 *FlashReg_Addr[ROBOT_ACTION_BYE_LEN];
//static u8   Flash_BUFF[16];
//static u16 Checksum = 0;

static u16 CheckSumFault = 0;


/*******************************************************************************
 * Routine     : Robot_Flash_Init
 * Function    : Robot Flash Initialization
 ******************************************************************************/
void Robot_Flash_Init(void)
{
	u32 tmp32 = 0;
	//u8   cnt = 0;

	for(tmp32=0;tmp32<ROBOT_ACTION_BYE_LEN;tmp32++)
	{FlashReg_Addr[tmp32] = 0;}

	FlashReg_Addr[0] = (u16*)&CheckSumFault;
	
	Robot_Flash_SPI1_Init();

	/*0x000H-0x00FH Flash Gear Action Parameter------------------------------------*/
	//Robot_Flash_Read(Flash_BUFF,u32 ReadAddr,16);  			
	//for(cnt=0;cnt<16;cnt++)    Checksum+=Flash_BUFF[cnt];
	/*Data to Action */
}

/*******************************************************************************
 * Routine     : Robot_Flash_SPI1_Init
 * Function    : Robot Flash SPI1 Initialization
 ******************************************************************************/
void Robot_Flash_SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE );
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

 	GPIO_SetBits(GPIOB,GPIO_Pin_5 |GPIO_Pin_6 |GPIO_Pin_7);  

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  	//����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;					//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;						//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;					//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;						//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;	//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ2  36/2=18MHz 
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  								//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); 										//ʹ��SPI����
	
	//SPI1_Read_Write_Byte(0xff);									//��������	
}   

/*******************************************************************************
 * Routine   : Robot_FLASH_Out
 * Function  : Flash out parm
 ******************************************************************************/
u16 Robot_FLASH_Out(u8 regist)
{  
	if(regist < 50)	return (*FlashReg_Addr[regist]);
	else			return(0);
}

/*******************************************************************************
 * Routine   : SPI1_ReadWriteByte
 * Function  : Read and Write SPI1 
 		      TxData write data and return data 1 byte ,wait time funtion
 ******************************************************************************/
u8 SPI1_Read_Write_Byte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry > OPERATION_TIMES)return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData); 
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry > OPERATION_TIMES)return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); 				    
}

/*******************************************************************************
 * Routine   : Robot_Flash_ReadID
 * Function  : Read Robot Flash ID = OxEF14,wait time funtion
 ******************************************************************************/
u16 Robot_Flash_ReadID(void)
{
	u16 id = 0;	  
			    
	SPI1_Read_Write_Byte(0x90);				// Send read ID command
	SPI1_Read_Write_Byte(0x00); 	    
	SPI1_Read_Write_Byte(0x00); 	    
	SPI1_Read_Write_Byte(0x00); 	 			   
	id|=SPI1_Read_Write_Byte(0xFF)<<8;  
	id|=SPI1_Read_Write_Byte(0xFF);	 
		    
	return id;
}   

/*******************************************************************************
 * Routine   : Robot_Flash_Read_SR
 * Function  : Read Robot Flash Status register,wait time funtion
 *BIT7  6   5   4   3   2   1   0
 *SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
 *TB,BP2,BP1,BP0:FLASH����д��������
 *WEL:дʹ������
 *BUSY:æ���λ(1,æ;0,����)
 *Ĭ��:0x00
 ******************************************************************************/
u8 Robot_Flash_Read_SR(void)   
{  
	u8 temp8=0;   
	
	SPI1_Read_Write_Byte(W25X_ReadStatusReg);    	//���Ͷ�ȡ״̬�Ĵ�������    
	temp8 = SPI1_Read_Write_Byte(0Xff);             		//��ȡһ���ֽ�  
	return temp8;   
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Write_SR
 * Function  :  Write Flash Status register,wait time funtion
 *BIT7  6   5   4   3   2   1   0
 *SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *Only SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2) can write
 ******************************************************************************/
void Robot_Flash_Write_SR(u8 sr)   
{    
	SPI1_Read_Write_Byte(W25X_WriteStatusReg);   		//����дȡ״̬�Ĵ�������    
	SPI1_Read_Write_Byte(sr);               			  	//д��һ���ֽ�    	      
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Write_Enable
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Write_Enable(void)   
{
    	SPI1_Read_Write_Byte(W25X_WriteEnable);      		//����дʹ��    	      
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Write_Disable
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Write_Disable(void)   
{  
   	SPI1_Read_Write_Byte(W25X_WriteDisable);     		//����д��ָֹ��       	      
} 	

/*******************************************************************************
 * Routine   : Robot_Flash_PowerDown
 * Function  :  Write Flash Power down register, wait time funtion
 ******************************************************************************/
void Robot_Flash_PowerDown(void)   
{ 
    SPI1_Read_Write_Byte(W25X_PowerDown);        		//���͵�������  	      
}   

/*******************************************************************************
 * Routine   : Robot_Flash_WAKEUP
 * Function  :  Write Flash Power up register, wait time funtion
 ******************************************************************************/
void Robot_Flash_WAKEUP(void)   
{  
    	SPI1_Read_Write_Byte(W25X_ReleasePowerDown);   
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Wait_Busy
 * Function  :  Write Flash Enable register, Set WEL bit, wait time funtion
 ******************************************************************************/
void Robot_Flash_Wait_Busy(void)   
{   
	//while (Robot_Flash_Write_SR(0x01)==0x01);   	// �ȴ�BUSYλ��� ?
} 

/*******************************************************************************
 * Routine   : Robot_Flash_Erase_Chip
 * Function  :  Erase Flash whole chip,  wait time funtion S 
 ******************************************************************************/
void Robot_Flash_Erase_Chip(void)   
{                       
	Robot_Flash_Write_Enable();
    	Robot_Flash_Wait_Busy();   
    	SPI1_Read_Write_Byte(W25X_ChipErase);        		//����Ƭ��������  
	Robot_Flash_Wait_Busy();   				   		//�ȴ�оƬ��������
}   

/*******************************************************************************
 * Routine   : Robot_Flash_Erase_Sector
 * Function  :  Erase Flash chip sector,  wait time funtion S 
 *����һ������
 *Dst_Addr:������ַ 0~511 for w25x16
 *����һ��ɽ��������ʱ��:150ms
 ******************************************************************************/
void Robot_Flash_Erase_Sector(u32 Dst_Addr)   
{   
	Dst_Addr*=4096;
	
    	Robot_Flash_Write_Enable();                
    	Robot_Flash_Wait_Busy();   
    	SPI1_Read_Write_Byte(W25X_SectorErase);      		//������������ָ�� 
    	SPI1_Read_Write_Byte((u8)((Dst_Addr)>>16));  	//����24bit��ַ    
    	SPI1_Read_Write_Byte((u8)((Dst_Addr)>>8));   
    	SPI1_Read_Write_Byte((u8)Dst_Addr);    	      
    	Robot_Flash_Wait_Busy();   				  
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Read
 * Function  :  Read Flash data, wait time funtion
 * Read Flash Fix Length data 
 * pBuffer:���ݴ洢��
 * ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
 * Len:Ҫ��ȡ���ֽ���(���65535)
 ******************************************************************************/
void  Robot_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 Len)   
{ 
	u16 cnt16 = 0;    
	
    	SPI1_Read_Write_Byte(W25X_ReadData);         		//���Ͷ�ȡ����   
    	SPI1_Read_Write_Byte((u8)((ReadAddr)>>16));  	//����24bit��ַ    
    	SPI1_Read_Write_Byte((u8)((ReadAddr)>>8));   
    	SPI1_Read_Write_Byte((u8)ReadAddr);   
    	for(cnt16=0;cnt16<Len;cnt16++)
	{ 
        	pBuffer[cnt16]=SPI1_Read_Write_Byte(0XFF);   //ѭ������  
    	}	      
}  

/*******************************************************************************
 * Routine   : Robot_Flash_Read
 * Function  :  Read Flash data, wait time funtion
 * Write Flash Fix Length data 
 * pBuffer:���ݴ洢��
 * ReadAddr:��ʼд��ĵ�ַ(24bit)
 * Len:Ҫ��ȡ���ֽ���(���65535),������Ӧ�ó�����ҳ��ʣ���ֽ���
 ******************************************************************************/
void Robot_Flash_Write_Page(u8* pBuffer,u32 WriteAddr,u16 Len)
{
	u16 cnt16 = 0;
	
    	Robot_Flash_Write_Enable();                  
    	SPI1_Read_Write_Byte(W25X_PageProgram);      			//����дҳ����   
    	SPI1_Read_Write_Byte((u8)((WriteAddr)>>16)); 		//����24bit��ַ    
    	SPI1_Read_Write_Byte((u8)((WriteAddr)>>8));   
    	SPI1_Read_Write_Byte((u8)WriteAddr);   
   	for(cnt16=0;cnt16<Len;cnt16++)SPI1_Read_Write_Byte(pBuffer[cnt16]);
	Robot_Flash_Wait_Busy();					   		//�ȴ�д�����
} 

