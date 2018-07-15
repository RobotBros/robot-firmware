/*******************************************************************************
 * FileName��Robot_Head_LED.c
 * Function: Robot Head LED drive function
 * Designer: Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/	    
#include "Robot_Head_LED.h"

void TIM4_PWM_Init(u16 arr, u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);						 						
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  

/*GPIOB.9 IPK Fault I/O-----------------------------------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;																				
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     				
	
/*TIM4 Set Frq-------------------------------------------------------------*/
	TIM_TimeBaseStructure.TIM_Period = arr; 											
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 										
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 							

/*TIM4 Set Op mode---------------------------------------------------------*/	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 			//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//�������:TIM����Ƚϼ��Ը�
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  		  				//����Tָ���Ĳ�����ʼ������TIM4_CH4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  			//ʹ��TIM4 CH4��CCR4�ϵ�Ԥװ�ؼĴ��� 
	
	TIM_Cmd(TIM4, ENABLE); 
}

/******************************************************************************
 * Routine     : Robot_Head_LED_PWM_Init
 * Function    : PWM For Head LED Initialization
 ******************************************************************************/
 void Robot_Head_LED_PWM_Init(void)
{
	 TIM4_PWM_Init(HEAD_PWM_DUTY_ARR,HEAD_PWM_DUTY_DIV);
	 Robot_Head_LED_PWM_Percentage(50);
}

/******************************************************************************
 * Routine     : Robot_Head_LED_PWM_Percentage
 * Function    : PWM Duty For TIM4 CCR --- 0%~50%
 ******************************************************************************/
 void Robot_Head_LED_PWM_Percentage(u8 percentage)
 {
 	u16 cnt16 = 0;

	if(percentage > HEAD_MAX_DUTY)
	{
		cnt16 = HEAD_PWM_DUTY_ARR * 50 / 100;
	}
	else
	{
		cnt16 = HEAD_PWM_DUTY_ARR * percentage / 100;
	}
	
	TIM_SetCompare4(TIM4,cnt16);
 }
