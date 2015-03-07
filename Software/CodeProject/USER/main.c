#include <stm32f10x_lib.h>
#include "../SYSTEM/sys.h"
#include "../SYSTEM/delay.h"
#include "../HARDWARE/init.h" 
#include "../HARDWARE/usart.h"

#include "option.h"

u16 set_temp;					//命令设定温度值1C°
u16 comp_temp;					//全局补偿温度值0.1C°
u16 each_seg_comp[CHANNEL];		//各节补偿温度值1C°
u8  duty_ratio_shadow[CHANNEL];	//各节继电器输出占空比
u8  duty_ratio[CHANNEL];		//各节继电器输出占空比（0~WORK_CYCLE）
u8  frame_space_time;			
u16	temp[CHANNEL];				//存放各通道转换后的温度值0.1C°


u16	ad_value[CHANNEL][SAMP_TIME+1]; //存放AD转换结果
u8	channel_num=0;				     //标志当前的模拟开关通道值
u8	curt_samp=0;				     //标志当前通道的当前采样次数 EOC中断计数
u8	curt_cycle=0;				     //标志定时器中断次数		 Timer中断计数
u8	samp_ok=0;					     //标志AD工作完成

u16	param[PARAM_NUM];				//掉电保护时保存的参数

#if _DEBUG
	u16  sample_time=0;				//标志通道SAMP_TIME次数据采集的工作时间
	u8   com_time=0;				//标志采样与控制过程结束
#endif

																													
uc16 table[] = { 			  		//Rt分度表，阻值扩大100倍化为整型

/*0*/	10000, 10039, 10078, 10117, 10156, 10195, 10234, 10273, 10312, 10351,   
/*10*/	10390, 10429, 10468, 10507, 10546, 10585, 10624, 10663, 10702, 10740,	  
/*20*/	10779, 10818, 10857, 10896, 10935, 10973, 11012, 11051, 11090, 11129,	  
/*30*/	11167, 11206, 11245, 11283, 11322, 11361, 11400, 11438, 11477, 11515,	  
/*40*/	11554, 11593, 11631, 11670, 11708, 11747, 11786, 11824, 11863, 11901,   
/*50*/	11940, 11978, 12017, 12055, 12094, 12132, 12171, 12209, 12247, 12286,   
/*60*/	12324, 12363, 12401, 12439, 12478, 12516, 12554, 12593, 12631, 12669,
/*70*/	12708, 12746, 12784, 12822, 12861, 12899, 12937, 12975, 13013, 13052, 
/*80*/	13090, 13128, 13166, 13204, 13242, 13280, 13318, 13357, 13395, 13433,
/*90*/	13471, 13509, 13547, 13585, 13623, 13661, 13699, 13737, 13775, 13813,
/*100*/	13851, 13888, 13926, 13964, 14002, 14040, 14078, 14116, 14154, 14191,
/*110*/	14229, 14267, 14305, 14343, 14380, 14418, 14456, 14494, 14531, 14569,
/*120*/	14607, 14644, 14682, 14720, 14757, 14795, 14833, 14870, 14908, 14946,
/*130*/	14983, 15021, 15058, 15096, 15133, 15171, 15208, 15246, 15283, 15321,
/*140*/	15358, 15396, 15433, 15471, 15508, 15546, 15583, 15620, 15658, 15695,
/*150*/	15733, 15770, 15807, 15845, 15882, 15919, 15956, 15994, 16031, 16068,
/*160*/	16105, 16143, 16180, 16217, 16254, 16291, 16329, 16366, 16403, 16440,
/*170*/	16477, 16514, 16551, 16589, 16626, 16663, 16700, 16737, 16774, 16811,
/*180*/	16848, 16885, 16922, 16959, 16996, 17033, 17070, 17107, 17143, 17180,
/*190*/	17217, 17254, 17291, 17328, 17365, 17402, 17438, 17475, 17512, 17549,
/*200*/	17586, 17622, 17659, 17696, 17733, 17769, 17806, 17843, 17879, 17916,
/*210*/	17953, 17989, 18026, 18063, 18099, 18136, 18172, 18209, 18246, 18282,
/*220*/	18319, 18355, 18392, 18428, 18465, 18501, 18538, 18574, 18611, 18647, 
/*230*/	18684, 18720, 18756, 18793, 18829, 18866, 18902, 18938, 18975, 19011, 
/*240*/	19047, 19084, 19120, 19156, 19192, 19229, 19265, 19301, 19337, 19374, 
/*250*/	19410, 19446, 19482, 19518, 19555, 19591, 19627, 19663, 19699, 19735
} ;																 


/***************************** 简单数字滤波函数(掐头去尾取平均值) *******************************/

u16 Filt(u8 N, u16* a)
{
	u8 i;
	u16  max, min;
	u16  sum, aver;

	max=a[0]; min=a[0]; sum=a[0];
	for(i=1; i<N; i++)
	{	 
	    sum += a[i];

	 	if( a[i] > max )
			max = a[i];
		if( a[i] < min )
			min = a[i];			
	}
	aver = ( sum-max-min ) / (N-2);
	return aver;
}

/***************************** 顺序查找温度值*****************************************************/

void TempSearch(u8 N, u16* a)
{
	u8  i2;
	u16 j;
	u16 r[CHANNEL],ad[CHANNEL];
	u32 k;

	for(i2=0; i2<N; i2++)				//CHANNEL
	{
		//对采样值进行滤波，之后求出电阻值
		//校准后的电阻值R与AD采样ad之间的关系为：R=0.0256573*ad+98.6598
		//在处理中避免使用浮点数，将小数转化为整形数计算		
		ad_value[i2][SAMP_TIME] = Filt(SAMP_TIME , &ad_value[i2][0]);   //去头去尾均值滤波
	    ad[i2] = ad_value[i2][SAMP_TIME];                               				
		k = (42037*ad[i2]) + 161644184;	//这里42307=2.56573*2^14,161644184=9865.98*2^14
		r[i2] = (u16)(k>>13);			//这里得到实际电阻的2*100倍
										//分度表中的电阻值为实际电阻值的100倍
										//线性插值计算式再扩大两倍，以减少计算损失精度

		if(r[i2] < 2*10000)		        //低于0度的都记为0度
		{								//上面的得到电阻值相对于分度表中的两倍，因此这里乘2
			temp[i2]  = 0;
			continue;
		}

		if(r[i2] > 2*19735)		    	//高于260度的都记为260度
		{								//上面的得到电阻值相对于分度表中的两倍，因此这里乘2
			temp[i2] = 2600;
			continue;
		}
		                               	//0~260度的阻值范围内，查表，线性插值
		for (j=1; j<260; j++)		   
		{
			if(2*table[j]>r[i2])
			{
				//做0.1度的线性插值，公式如下
				//(Temp-10*(j-1))/(r-2*table[j-1]) = (10*j-10*(j-1))/(2*table[j]-2*table[j-1])
				temp[i2] = 10*(j-1) + 5*(r[i2]-(table[j-1]<<1))/(table[j]-table[j-1]);
				break;	
			}	   
	    }
	   temp[i2] -= ( ((short)each_seg_comp[i2]-5)*10 + comp_temp*i2 );
	}
}

/*****************************加热继电器输出控制****************************
*该函数根据当前温度与设定温度之间的差值设定加热继电器加热占空比的值
*占空比值取值范围为：0~100，代表每100ms之间继电器合上的时间
*备注：具体控制参数需现场测定
***************************************************************************/																							  
void HeaterCtr()
{
	u8 i;
	u16 temp_min = (set_temp-1)*10;
	u16 temp_max = (set_temp+1)*10;

	for(i=0; i<CHANNEL; i++)
	{
		duty_ratio_shadow[i] = 50;		//在设定温度的一定范围之间时，进行保温处理

	    if ( temp[i] > temp_max )		//高于设定温度一定范围时，降低继电器控制占空比以降温
			duty_ratio_shadow[i] = 20;

		if ( temp[i] < temp_min )		//低于设定温度一定范围时，增大继电器控制占空比以加热
			duty_ratio_shadow[i] = 85;
	} 			 
}

/********************************多路开关控制  通道切换 ********************************************/

void SwitchChannel (u8 key_num)
{  
	if (key_num<16)
	{	
	    GPIOB->BSRR |= 1<<15;	 //多路开关 禁止输出
	    GPIOB->BRR  |= ( 1<<11 | 1<<12 | 1<<13 | 1<<14);

	   	if( key_num & 0x01 )
		GPIOB->BSRR |= 1<<11;
	    if( key_num	& 0x02 )
		GPIOB->BSRR |= 1<<12;
	    if( key_num	& 0x04 )
		GPIOB->BSRR |= 1<<14;
		if( key_num	& 0x08 )
		GPIOB->BSRR |= 1<<13;
	
		GPIOB->BRR |= 1<<15;     //开启开关
	}
	else
	return;
}
/******************************  定时器中断函数******************************************/

void TIM3_IRQHandler(void)
{
	u16 i=0;   
	TIM3->SR &= ~(1<<0);                //清中断标志位
	curt_cycle++;	

	if (curt_cycle <= CHANNEL)		    //定时器前N个周期对各个通道采样
	{	       						
		ADC1->SR  &= ~(1<<1);           //清EOC转换结束标志
		ADC1->CR1 |=1<<5;		        //开EOC中断
	}

	if(curt_cycle == WORK_CYCLE)		//一个工作周期结束
	{
		curt_cycle = 0;

		//将加热继电器占空比映射数组中的值装载到占空比数组中，每100ms执行一次
		for(i = 0;i < CHANNEL;i ++)
		{
			duty_ratio[i] = duty_ratio_shadow[i];
		}
	}
}
								 
/***********************  ADC（此处只用PA1做单通道转换）中断函数*******************/

void ADC_IRQHandler()
{
	ad_value[channel_num][curt_samp] = ADC1->DR;//转换结束标志位EOC在读ADC->DR时清除
    curt_samp+=1;						     //计数EOC中断次数，标志当前通道采样次数

    if(curt_samp == SAMP_TIME)			   	 //当前通道的n次采样结束
    {	
   	    curt_samp = 0; 
		channel_num = (channel_num+1)%CHANNEL;

		ADC1->CR1 &=~(1<<5);		         //关EOC中断
		ADC1->SR &= ~(1<<1);                 //清EOC转换结束标志
		SwitchChannel (channel_num);         //切换通道

		if(channel_num == 0)
		samp_ok = 1;

#if _DEBUG
	   	sample_time = TIM3->CNT;
#endif

    }
}
/***********************  掉电保护外部中断函数***********************************/
 void EXTI9_5_IRQHandler (void)
{
	u8 i=0;
	if(  EXTI->PR&(1<<8)  )
	{
		EXTI->PR |= 1<<8;	            //写1清零
		FLASH_Unlock();	  				//对内部FLASH操作，需要先解锁

		//先写入异常标志，然后存入参数
		FLASH_ProgramHalfWord((PARAM_ADDR+(PARAM_NUM<<1)),0x1234);
		for(i=0;i<PARAM_NUM;i++)		//将参数存入Flash中
		{
			//调用写操作,将参数写入Flash中
			FLASH_ProgramHalfWord((PARAM_ADDR+(i<<1)),param[i]);
		}
		FLASH_Lock();					//操作完之后上锁
	}
} 
/**********************************主函数************************************/
int main(void)
{
	u8 i=0;	
	Stm32ClockInit(9);                      //系统时钟初始化，PLLCLK=8M*9=72M 
	DelayInit(72);

	GPIOConfig();
	EXTI8_Init();
	Usart1_Init(72,BAUDRATE);				//USART1初始化 USART1:pclk2
	Timer3_Init(800*SAMP_TIME, 8);			//72M/(8+1)=8Mhz的计数频率 eg:计数到8000为8000/8M=1ms
	ADC1_Init();
   	SwitchChannel (0); 
	ADC1->CR2 |= 5<<20;						//SWSTART触发规则通道转换 AD转换放在定时器中断里开

	for(i=0;i<PARAM_NUM;i++)
	{
		param[i] = 0xffff;	
	}

	if(*(vu16*)(PARAM_ADDR+(PARAM_NUM<<1)) != 0xffff)
	{
		if(*(vu16*)(PARAM_ADDR+(PARAM_NUM<<1)) == 0x1234)
		{
			for(i=0;i<PARAM_NUM;i++)
			{
				param[i] = *(vu16*)(PARAM_ADDR + (i<<1));	
			}
		}

		FLASH_Unlock();	  				//对内部FLASH操作，需要先解锁
		i = FLASH_ErasePage((u32)PARAM_ADDR);
		FLASH_Lock();					//操作完之后上锁

#if	_DEBUG
		if(i != FLASH_COMPLETE)
		{
			USART1_Print("Flash erase error!\n");
		}
#endif
	}

#if	_DEBUG
	for(i=0;i<PARAM_NUM;i++)
	{
		param[i] = param[i]+2*i+1;	
	}
#endif

	comp_temp = 0;							//补偿温度初值
	i = 0;
	while(i<CHANNEL)
    {
		each_seg_comp [i] = 5;				//各节补偿温度初值0x05=0C°
		duty_ratio_shadow[i]= WORK_CYCLE ;	//各节继电器输出占空比初值EachDutyRatio[i]/WORK_CYCLE=1
		duty_ratio[i] = duty_ratio_shadow[i];
		i++;
	}

	while(1)
	{
		for(i=0; i<CHANNEL; i++)	  
		{
			if( curt_cycle < duty_ratio[i] )
				GPIOC->BSRR |= 1<<i;
			else
				GPIOC->BRR  &= ~(1<<i);
		}

		if ( samp_ok == 1 )	               //采样结束。后几个周期不开AD，用来查表转换温度值，或继电器输出，通讯
		{
			samp_ok = 0;
			TempSearch(CHANNEL,temp);
			HeaterCtr();

#if _DEBUG
	    	com_time = curt_cycle;
#endif

			ComunicatProcess();
		}		  
	}
}

					