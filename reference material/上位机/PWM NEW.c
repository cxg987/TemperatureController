
/* 左面、右面运行控制（快慢速控制，快慢速时间） */
#include <stc_cpu.H>
#include <intrins.h>
#include <stdio.h>
#include <PWM_NEW.h>


#define U8 unsigned char
#define U16 unsigned int


// pwm初始化

void  Pwm_Init()
{
	CCON = 0;		   //清所有中断标志，timer stop running
	CMOD = 0X02;	   //set PCA/PWM timer clock source as SYCCLK/2

	CL = 0;		   // set PCA base timer
	CH = 0;

	//CCAP0H = CCAP0L = 0x80;		 //PWM0 占空比50%
	CCAPM0 = 0X42;			 //PWM0 使能

	//CCAP1H = CCAP1L = 0x80;	     //PWM1 占空比50%
	CCAPM1 = 0X42;			 //PWM1 使能

    CR = 1;					 //	 start timer

}






