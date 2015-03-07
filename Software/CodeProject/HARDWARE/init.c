#include "init.h"
#include <stm32f10x_lib.h>

/***************************** 通用IO口初始化*****************************************************/

void GPIOConfig(void)
{  	
   
	RCC->APB2ENR  |= (1<<2 |1<<3 | 1<<4);  //使能PORTA,PORTB,PORTC时钟,
	RCC->APB2RSTR |= (1<<2 |1<<3 | 1<<4);  //A,B,C端口复位
	RCC->APB2RSTR &= ~(1<<2 |1<<3 | 1<<4);

	/* 拨码开关 按键PA8,9,10,11初始化 */
	GPIOA->CRH &=0xFFFF0000;
	GPIOA->CRH |=0x00008888;             //PA8,9,10,11状态设置输入
	GPIOA->ODR |=0x00000F00;	         //PA8.9.10.11输出上拉 ? (经测试输出不上拉就无法正确输入)
	GPIOA->IDR |=0x00000F00;			 //输入寄存器置高  因按键按下时是低电平

    /* LED 初始化PC0~PC15 */
	GPIOC->CRL = 0x11111111;			 //PC0~PC15 通用推挽输出 10MHz
	GPIOC->CRH = 0x11111111;
	GPIOC->ODR = 0;

	/* 多路模拟开关控制 PB11(A0)，PB12(A1)，PB14(A2), PB13(A3), PB15(输出使能)*/
    GPIOB->CRH &= 0x00000FFF;
	GPIOB->CRH |= 0x11111000;			 //PC0~PC15 通用推挽输出 10MH
	GPIOB->ODR |= 1<<15	;				 //PC15置高，禁止多路开关输出
}

/******************************  定时器初始化******************************************/

void Timer3_Init(u16 arr, u16 psc)
{
	RCC->APB1ENR |= 1<<1;  //使能TIM3时钟 ,APB1 最大36MH
	TIM3->ARR = arr;	   //设定定时器的自动重装载值
	TIM3->PSC = psc;       //预分频值3600，得到10Khz的计数时钟 计数器的时钟频率(CK_CNT)等于fCK_PSC/( PSC[15:0]+1)。

	TIM3->DIER |= 1<<0;	   //允许更新中断
	TIM3->CR1 |= 0x01;     //使能定时器3
	MY_NVIC_Init(1, 0, TIM3_IRQChannel, 2);//抢占1，子优先级0，组2
}

/***********************  ADC初始化 （此处只用PA1做单通道转换）*******************/
			   
void ADC1_Init()
{
	RCC->APB2ENR |= 1<<2;		   //使能PORTA时钟
	GPIOA->CRL &= 0xFFFFFF0F;	   //PA1模拟输入

	RCC->CFGR |= 2<<14;			   //ADCCLK（最大14Mh）=PCLK2 6分频后 72M/6=12M
	RCC->APB2ENR  |= 1<<9;         //使能ADC1时钟
	RCC->APB2RSTR |= 1<<9;         //ADC1接口复位
   	RCC->APB2RSTR &= ~(1<<9);      //停止ADC1接口复位    这样做可以将ADC1的寄存器都复位后续配置时不用先清，可直接赋值

	ADC1->CR2  |= (1<<1);		    // CONT=1 连续转换
	ADC1->CR2  |=  7<<17 ;          //用SWSTART软件触发转换，右对齐，不用DMA

/*	ADC1->CR1  |=  1<<8;		    //规则组扫描模式
	ADC1->SQR1 |=  (10-1)<<20;      //SAMP-FREQ个规则通道转换(这里是每路模拟开关通道的转换次数)，
	ADC1->SQR3 |=  (1<<0 | 1<<5 | 1<<10 | 1<<15 | 1<<20 | 1<<25); //规则转换序列的第1~6的通道号均为INT2
	ADC1->SQR2 |=  (1<<0 | 1<<5 | 1<<10 | 1<<15 | 1<<20 | 1<<25); //规则转换序列的第7~12的通道号均为INT2
	ADC1->SQR1 |=  (1<<0 | 1<<5 | 1<<10 | 1<<15 );				  //规则转换序列的第13~16的通道号均为INT2 */

	ADC1->SQR1 |=  (1-1)<<20;	    //1个规则通道转换
	ADC1->SQR3 |=  1<<0;
	ADC1->SMPR2 |= (4<<3);          //通道1 采样周期41.5个cycle   完成一次转换（12.5+41.5）/12M=4.5us

    ADC1->CR2 |= 1<<0;               //ADON由0置1，AD转换器上电, 每次上电后进行校准
	ADC1->CR2 |= 1<<3;			     //复位校准值
	while( ADC1->CR2 &(1<<3) == 0);								   
	ADC1->CR2 |= 1<<2;			     //开始校准
	while( ADC1->CR2 &(1<<2) == 0);
	
	ADC1->CR1 &= ~(1<<5);		             //关EOC中断
	MY_NVIC_Init(3,2,ADC1_2_IRQChannel,2);	 //组2里 抢占优先级3，相应优先级3
				
}

/******************************  掉电外部中断初始化******************************************/
void EXTI8_Init(void)
{
	RCC->APB2ENR|=1<<3;        //使能PORTB时钟
	RCC->APB2ENR|=1<<0;        //开启AFIO辅助时钟

	GPIOB->CRH &= 0XFFFFFFF0;  //PB8设置成输入	  
	GPIOB->CRH |= 0X00000008; 				   
	GPIOB->ODR |= 1<<8;	       //PB8上拉

	AFIO->EXTICR[2]|= 1;       //EXTI.8映射到GPIOB.BIT8
	EXTI->IMR   |= 1<<8;       //开启line BIT8上的事件
	EXTI->FTSR  |= 1<<8;	   //line BIT8上事件下降沿触发

	MY_NVIC_Init(0,0,EXTI9_5_IRQChannel,2);    //抢占0，子优先级0，组2	   
}

