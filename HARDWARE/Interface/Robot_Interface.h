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
extern uint16 GetMotorCurrent(void); // 获取电机电流，返回值为实际值*10，单位为A
extern uint16 GetPfcCurrent(void);   // 获取PFC电流，返回值为实际值*10，单位为A
extern uint16 GetAcVoltage(void);   // 获取AC电压值，返回值为实际值，单位为V
extern uint16 GetDcVoltage(void);   // 获取DC电压值，返回值为实际值，单位为V
extern uint16 GetMotorSpeed(void);  // 获取压缩机转速，返回值为机械转速，单位为RPM
extern uint8 GetMotorStatus(void);  // 获取压缩机运行状态，0为待机，1为运行，2为故障 ，3预加热，4自检
extern uint8 GetDriverFaultcode(void);//获取驱动故障代码，参考产品规格书
extern uint16 adc_get_data(uint8 channel);

#define	CHANNEL_OCT	0  // CHANNEL_OCT
#define	CHANNEL_CCT	1  // CHANNEL_CCT
#define	CHANNEL_OAT	2  // CHANNEL_OAT
/*Function read for KELON-----------------------------------------------------*/
extern void fltclr(void);	// 故障清除，部分驱动故障不会自动清除，需主控清除
extern void Motor_OnOff(uint8 RunCmd); //RunCmd为开关机指令，1为开机，0为关机，2为预加热
extern void Motor_Speed(uint16 TargetSpeed); //TargetSpeed为目标转速，为机械转速，单位是RPM；
extern void Motor_AccelRate(uint16 AccelRate); //AccelRate为升斜率，单位为RPM/S
extern void Motor_DecelRate(uint16 DecelRate); //DecelRate为降斜率，单位为RPM/S
//extern void Motor_Pfc(uint8 PfcCmd); //PfcCmd为PFC开关，1为开启，0为关闭
extern void Motor_Torque(uint8 Cmd);  // 1为开启力矩补偿，0为关闭力矩补偿
extern uint8 Receive_Data(void);
extern void Send_Data(uint8 temp);
extern void UartPC_RxInterrupt(void) ;
extern void UartPC_TxInterrupt(void) ;

/*Function for KELON I2C Communication----------------------------------------*/
// BufAddr: 数据数组地址
// E2WrAddr: E2器件写地址，参考E2数据手册
// WordAddr: 字地址
// DataCnt: 顺序读取字节个数
extern void Driver_Write_E2_GPIO(uint8 *BufAddr, uint8 E2WrAddr,uint8 WordAddr, uint8 DataCnt);//写E2程序
extern void Driver_Read_E2_GPIO(uint8 *BufAddr, uint8 E2WrAddr,uint8 WordAddr, uint8 DataCnt);//读E2程序


#endif