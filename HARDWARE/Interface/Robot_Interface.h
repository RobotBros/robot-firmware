#ifndef ROBOT_INTERFACE_H
#define  ROBOT_INTERFACE_H

#include "Public.h"
#include "XMC1400.h"
#include "XMC1400_GPIO.h"

/*Function declaration--------------------------------------------------------*/
extern void SDHET_Init(void);
extern void SDHET_Main(void);
extern void Driver_Init(void);
extern void Driver_Main(void);
extern void Inverter_Init(void);
extern void Inverter_Main(void);
extern void SDHET_TMR_1MS(void);

/*Function read for KELON-----------------------------------------------------*/
extern uint16 GetMotorCurrent(void); // ��ȡ�������������ֵΪʵ��ֵ*10����λΪA
extern uint16 GetPfcCurrent(void);   // ��ȡPFC����������ֵΪʵ��ֵ*10����λΪA
extern uint16 GetAcVoltage(void);   // ��ȡAC��ѹֵ������ֵΪʵ��ֵ����λΪV
extern uint16 GetDcVoltage(void);   // ��ȡDC��ѹֵ������ֵΪʵ��ֵ����λΪV
extern uint16 GetMotorSpeed(void);  // ��ȡѹ����ת�٣�����ֵΪ��еת�٣���λΪRPM
extern uint8 GetMotorStatus(void);  // ��ȡѹ��������״̬��0Ϊ������1Ϊ���У�2Ϊ���� ��3Ԥ���ȣ�4�Լ�
extern uint8 GetDriverFaultcode(void);//��ȡ�������ϴ��룬�ο���Ʒ�����
extern uint16 adc_get_data(uint8 channel);

#define	CHANNEL_OCT	0  // CHANNEL_OCT
#define	CHANNEL_CCT	1  // CHANNEL_CCT
#define	CHANNEL_OAT	2  // CHANNEL_OAT
/*Function read for KELON-----------------------------------------------------*/
extern void fltclr(void);	// ��������������������ϲ����Զ���������������
extern void Motor_OnOff(uint8 RunCmd); //RunCmdΪ���ػ�ָ�1Ϊ������0Ϊ�ػ���2ΪԤ����
extern void Motor_Speed(uint16 TargetSpeed); //TargetSpeedΪĿ��ת�٣�Ϊ��еת�٣���λ��RPM��
extern void Motor_AccelRate(uint16 AccelRate); //AccelRateΪ��б�ʣ���λΪRPM/S
extern void Motor_DecelRate(uint16 DecelRate); //DecelRateΪ��б�ʣ���λΪRPM/S
//extern void Motor_Pfc(uint8 PfcCmd); //PfcCmdΪPFC���أ�1Ϊ������0Ϊ�ر�
extern void Motor_Torque(uint8 Cmd);  // 1Ϊ�������ز�����0Ϊ�ر����ز���
extern uint8 Receive_Data(void);
extern void Send_Data(uint8 temp);
extern void UartPC_RxInterrupt(void) ;
extern void UartPC_TxInterrupt(void) ;

/*Function for KELON I2C Communication----------------------------------------*/
// BufAddr: ���������ַ
// E2WrAddr: E2����д��ַ���ο�E2�����ֲ�
// WordAddr: �ֵ�ַ
// DataCnt: ˳���ȡ�ֽڸ���
extern void Driver_Write_E2_GPIO(uint8 *BufAddr, uint8 E2WrAddr,uint8 WordAddr, uint8 DataCnt);//дE2����
extern void Driver_Read_E2_GPIO(uint8 *BufAddr, uint8 E2WrAddr,uint8 WordAddr, uint8 DataCnt);//��E2����


#endif