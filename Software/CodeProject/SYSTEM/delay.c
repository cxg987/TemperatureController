 #include <stm32f10x_lib.h>
 #include "delay.h"

 /******************************************************
* 使用SysTick的普通计数模式对延迟进行管理
* 包括delay_us, delay_ms
*******************************************************/

static u8  fac_us=0;   //us延时倍乘数
static u16 fac_ms=0;   //ms延时倍乘数

 /******************************************************
* 初始化延时函数
* SYSTICK的时钟固定为HCLC时钟的1/8
* SYSCLK: 系统时钟频率，单位为HZ
*******************************************************/

void DelayInit(u8 SYSCLK)
{
	 SysTick->CTRL&=0xfffffffb;  //bit2置0，选择外部时钟 HCLK/8
	 fac_us=SYSCLK/8;
	 fac_ms=(u16)fac_us*1000;
}

 /******************************************************
* 延时nus函数
*******************************************************/

void DelayUs(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us;  //加载计数值
	SysTick->VAL=0x00;		   //清空计数器
	SysTick->CTRL=0x01;		   //开始倒数
	
	do
	{
	   temp=SysTick->CTRL;
	}
	while(temp&0x01 && !(temp&1<<16)); //等待时间到达
	SysTick->CTRL=0x00;		  //关闭计数器
	SysTick->VAL =0x00;		  //清空计数器
}

 /******************************************************
* 延时nms函数
* 注意nms的范围
* SysTick->LOAD为24位寄存器,所以,最大延时为:
* nms<=0xffffff*8/SYSCLK*1000
* SYSCLK单位为Hz,nms单位为ms
* 对72M条件下,nms<=1864 
*******************************************************/

void DelayMs(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;	//计数值加载(SysTick->LOAD为24bit)
	SysTick->VAL=0x00;			    //清空计数器
	SysTick->CTRL=0x01;				//开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器				   
}
