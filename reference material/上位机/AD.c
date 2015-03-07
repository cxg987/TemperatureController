#include <STC_CPU.H>
#include"intrins.h"
#include"math.h"
#include <String.h>
#include"uart.h"
#include"AD.h"
void AD_init();
void AD_work( BYTE val,BYTE channel);
sbit REL0 = P2^4;
sbit REL1 = P2^5;
extern u16 g_usRegister[66]; 
BYTE g_ucChn; // 当前采集的通道

//------------------------------------------------------------------------------
void AD_init()
{
	P1ASF=0x03;         //P1.0和P1.1口作为模拟功能A/D使用
    AUXR1|=0x04;
	ADC_RES=0;         //清零转换结果寄存器高2位
	ADC_RESL=0;       //清零转换结果寄存器低8位
	ADC_CONTR=0x80;  //开启AD电源
	IE |= 0xa0;      //允许AD中断
	g_ucChn = 0;
	delay(2);       //等待1ms，让AD电源稳定
	ADC_CONTR = 0x88 | g_ucChn;
}	

void adc_isr() interrupt 5 using 1
{
 	ADC_CONTR &= !ADC_FLAG;
	g_usRegister[24+g_ucChn]= (u16)ADC_RES<<10+(u16)ADC_RESL;
    if (g_ucChn == 0 ) g_ucChn = 1;
	else g_ucChn = 0;
	ADC_CONTR = 0x88| g_ucChn;
}

/*unsigned int AD_get(BYTE channel)
{
   ADC_CONTR=0x88|channel;    //开启AD转换1000 1000 即POWER SPEED1 SPEED0 ADC_FLAG   ADC_START CHS2 CHS1 CHS0 
   _nop_(); 
   _nop_(); 
   _nop_();
   _nop_();                  //要经过4个CPU时钟的延时，其值才能够保证被设置进ADC_CONTR 寄存器
   while(!(ADC_CONTR&0x10));    //等待转换完成
   ADC_CONTR&=0xe7;      //关闭AD转换，ADC_FLAG位由软件清0
   return((u16)ADC_RES<<10+(u16)ADC_RESL);   //返回AD转换完成的10位数据(16进制)
}
   */
void AD_work( BYTE val,BYTE channel)
{
	u16 AD_val;
	u16 js_vol,chazhi;
	AD_val=g_usRegister[24+channel];//
	js_vol= val >> 2;
	if(AD_val>js_vol)							
	chazhi=AD_val-js_vol;
	else
	chazhi=js_vol-AD_val;

	if(chazhi>100)
	{
		 REL0=0;
	}
}

