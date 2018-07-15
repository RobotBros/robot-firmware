/*******************************************************************************
 * FileName��Robot_LPF.c
 * Function:   Robot Low pass filter
 * Designer:  Fishcan/2018.05.08
 * MCU: STM32F103 series
 ******************************************************************************/
 
/*��ͨ�˲�����
һ�׹��Ի��ڵ���ѧ���ʽΪ��T*dc(t)/dt +c(t) =r(t),ȡ��ʽ�任����TsC(s)+C(s) =R(s),C(s)=R(s)/(Ts +1),��һ��RC��ͨ�˲����Ĵ��ݺ���Ϊ��
G(s) =C(s)/R(s) =1/(Ts +1)
ʽ�У�C(t)Ϊ���������r(t)Ϊ���������TΪʱ�䳣����
һ��һ�׵�ͨ�˲����ݺ���(RC�����˲�)��
(1) H(jw)=1/(1+jWRC) 
(2) Wc = 1/RC (��ֹƵ��) Wc = 2*pi*fc 
    H(jw) = 1/ SQR(1+(W/Wc)^2)
    W<Wc ��ֵ��˥������λ0
    W>Wc ��ֵһֱ˥�� ��λ-90
    W=Wc ��ֵ˥����0.707Vin ��λ-45
����������ɢ����Z�任��ַ���Ϊ��ǰ��ַ�������ַ�������������ǰ��ַ��ȶ��Խϲ��һ������ȶ��ԽϺõ�����ַ���
(1) H(s) = 1/(RCs+1) 
(2) s={1-z(-1)}/T T:��������
(3) Y(z)/X(z) = T/(T+RC)*X(z)+RC/(T+RC)*Y[z]*z(-1)  ==> X(z)=(1+RC/T)*Y(z)-RC/T*Y(z)*Z(-1) => X(n)=(1+RC/T)*Y(n)-RC/T*Y(n-1)
(4) Y[n]/X[n] = T/(T+RC)*X(n)+RC/(T+RC)*Y[n-1]
������ַ���Ϊ��
(1) Y[n] = Kc*X[n] +(1-Kc)Y[n-1] = Kc*(X[n]-Y[n-1])+Y[n-1]
(2) Kc = T/(T+RC) = 1/(1+��/T) ��:ʱ�䳣��
(3) Wc = 1/RC  ==> Kc = T/(T+1/Wc) = Wc/(Wc+f)  
ʽ�У�Xn   �����β���ֵ
      Yn-1 ���ϴε��˲����ֵ
     a(Kc) ���˲�ϵ������ֵͨ��ԶС��1
	Yn �������˲������ֵ
	f  : ����Ƶ��
*/

#include "Robot_LPF.h"

/***************************************************************************
 * Routine     : ��ͨ�˲�����λ�������ڹ̶��˲������Ȳ��ߵĳ���
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)={x(n)-y(n-1)}>>N/s
                 T = (2^N -1)*TsLPF    2^N=f/Wc+1
 * Input       : Xn,N
 * Run Time    : 30 cycles (Not Optimized 33 cycles)
 ***************************************************************************/
s16 LPFilt_2N(s16 In_data, tFilt *Lpf)
{
	s16 tmp16;
	
	if(Lpf->Enable) 
	{
		tmp16  = In_data - Lpf->YnLst;  // x(n) - y(n-1)
		Lpf->IntSum = Lpf->IntSum + tmp16 ;
		Lpf->Yn  = Lpf->IntSum >> Lpf->N;
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return (Lpf->Yn);
}

/***************************************************************************
 * Routine     : ��ͨ�˲��������ڹ̶��˲�����ĳ���
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)
                 K = 2^15*Wc/(Wc+f)  k = 0~1
 * Input       : Xn,K
 * Run Time    : 31 cycles (Not Optimized 36 cycles)
 ***************************************************************************/
s16 LPFilt_WcTs(s16 In_data, tFilt *Lpf)
{
	s32 tmp32;
	s16 tmp16;

	if(Lpf->Enable)
	{
		tmp16 = In_data-Lpf->YnLst;
		Lpf->IntSum =Lpf->IntSum +tmp16;
		tmp32 =Lpf->IntSum*Lpf->K;
		Lpf->Yn =(s16)(tmp32>>15);
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return(Lpf->Yn);
}

/***************************************************************************
 * Routine     : ��ͨ�˲��������ڱ仯�˲�����ĳ���
 * Function    : Low Filter with shift N 
 *               y(n)={x(n)-y(n-1)}*K+y(n-1)
                 K = Wc/(Wc+f)   f=Hz
 * Input       : Xn,Wc,FHz
 * Run Time    : 53 cycles (Not Optimized 59 cycles)
 ***************************************************************************/
s16 LPFilt_Wc(s16 In_data, tFilt *Lpf)
{
	s32 tmp32;
	s16 tmp16;
	
	if(Lpf->Enable)
	{	 
		tmp16 = In_data - Lpf->YnLst;  
		Lpf->IntSum = Lpf->IntSum + tmp16 ;
		tmp32 = Lpf->Wn;
		tmp32 = tmp32<<15;
		tmp16 = Lpf->Wn + Lpf->FHz;
		tmp32 = tmp32/tmp16;
		tmp32 = tmp32*Lpf->IntSum;
		Lpf->Yn = tmp32>>15;
		Lpf->YnLst = Lpf->Yn;
	}
	else
	{
		Lpf->IntSum = 0;
		Lpf->YnLst = 0;
		Lpf->Yn = 0;
	}
	return (Lpf->Yn);
}
