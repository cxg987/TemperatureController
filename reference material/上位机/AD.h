#ifndef _AD_H_11_08_30_
#define _AD_H_11_08_30_
#define uchar unsigned char

 /*define ADC operation const for ADC_CONTER*/


#define ADC_POWER 0x80;		//ADC power control bit 	
#define ADC_FLAG 0x10;		//ADC complete flag
#define ADC_START 0x08;		//ADC atsrt control bit
#define ADC_SPEEDLL 0x00;	//540 clocks
#define ADC_SPEEDL 0x20;	//360 clocks
#define ADC_SPEEDH 0x40;	//180 clocks
#define ADC_SPEEDHH 0X60;	//90 clocks

void AD_init();


#endif