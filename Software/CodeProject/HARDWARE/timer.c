#include "timer.h"

/***********************************************************
* 通用定时器 驱动代码
* 包含定时器初始化程序， 定时器3终端服务子程序
* 2ms 中断一次
************************************************************/

void Timerx_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1<<1;  //使能TIM3时钟
	TIM3->ARR = arr;	   //设定定时器的自动重装载值
	TIM3->PSC = psc;       //预分频值7200，得到10Khz的计数时钟 计数器的时钟频率(CK_CNT)等于fCK_PSC/( PSC[15:0]+1)。

	TIM3->DIER |= 1<<0;	   //允许更新中断
	TIM3->DIER |= 1<<6;	   //允许触发中断

	TIM3->CR1 |= 0x01;     //使能定时器3
	MY_NVIC_Init(1, 3, TIM3_IRQChannel, 2);//抢占1，子优先级3，组2
}

void TIM3_IRQHandler(void)
{
    u16 data;
	u8  i;
    if(TIM3->SR&0X0001)     //溢出中断
	{
//	   /*for(i=4; i<16; i++)
//		{
//			data=1<<i;
//			write_595(data);
//			
//		} */   
//	   data=LED5;
//	   write_595(data);
//	   delay_ms(500);
	}
	TIM3->SR &= ~(1<<0);    //清楚中断标志位	
}
