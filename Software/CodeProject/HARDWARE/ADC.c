#include "ADC.H"

#define  ADC1_DR_Address    ((u32)0x4001244C)
vu16 ADCConvertedValue;
int AD_value;
/******************************************************
* ADC驱动
包括 AD1初始化，DMA初始化，和串口发送转换结果
*******************************************************/
/*******************************************************************************
* 函数名  	: ADC1_Init
* 函数功能	: ADC1初始化
* 输入		: 无
* 输出		: 无
* 返回		: 无
*******************************************************************************/
//ADC1初始化
#define EN_ADC1_IRQ		//开ADC中断
			   
void ADC1_Init()
{
	RCC->APB2ENR |= 1<<2;				//使能PORTA时钟
	GPIOA->CRL &= 0xFFFFFF0F;
	GPIOA->CRL |= 0x00000000;			//PA1模拟输入

	RCC->CFGR |= 2<<14;					//ADCCLK（最大14Mh）=PCLK2 6分频后 72M/6=12M
	RCC->APB2ENR |= 1<<9;				//使能ADC1时钟
	RCC->APB2RSTR |= 1<<9;				//ADC1接口复位
   	RCC->APB2RSTR &= ~(1<<9);			//停止ADC1接口复位,
										//这样做可以将ADC1的寄存器都复位,
										//后续配置时不用先清,可直接赋值

	ADC1->CR1 |= 1<<8;					//扫描模式,
										//在所有通道上禁用模拟看门狗
										//禁止EOCIE

	ADC1->CR2 |= (7<<17 | 1<<20);		//用SWSTART触发转换，使用外部触发
	ADC1->CR2 |= 1<<8;					//使用用DMA
										//单次转换,右对齐

	ADC1->SQR1 |= (10-1)<<20;			//10个规则通道单次转换

	ADC1->SQR3 |= (1<<0 |1<<5 |1<<10);
	ADC1->SQR3 |= (1<<15 | 1<<20 | 1<<25);
	ADC1->SQR2 |= (1<<0 | 1<<5 | 1<<10 | 1<<15);
										//设置10个1通道的规则采样组

	ADC1->SMPR2 |= 4<<3;				//通道1采样周期41.5个cycle 

    ADC1->CR2 |= 1<<0;					//ADON由0置1，AD转换器上电, 每次上电后进行校准
	ADC1->CR2 |= 1<<3;					//复位校准值
	while( ADC1->CR2 &(1<<3));			//查询校准寄存器是否已初始化
	ADC1->CR2 |= 1<<2;					//开始校准
	while( ADC1->CR2 &(1<<2));			//查询校准是否已完成
	
//#ifdef EN_ADC1_IRQ
//	ADC1->CR1 |=1<<5;		//开EOC中断
//	MY_NVIC_Init(3,2,ADC1_2_IRQChannel,2);	 //组2里 抢占优先级3，相应优先级3
//#endif
//		   					  
//	ADC1->CR2 |= 5<<20;            //SWSTART触发规则通道转换			
}

//查询方式获得AD转换结果
  int GET_AD_Data()
{
     int data;
  
  	while ((ADC1->SR & (1<<1))==0)	//查询SR中的EOC	位
	{	     
	     data = ADC1->SR  ;
         USART1_SendByte(data); 
		 USART1_SendByte('M');
	 }
	data = (ADC1->DR&0x0fff);
    USART1_TXword(data);
	return  data ;
} 
/*******************************************************************************
* Function Name  : Time_Inter_Adjust
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DMA1_Init()

{	
	//复位DMA1_Channal1的各寄存器	 
	DMA1_Channel1->CCR &= ~(1<<0);	 // first Disable the selected DMAy Channelx 
  
  	DMA1_Channel1->CCR  = 0; 	     //Reset DMAy Channelx control register 
  	DMA1_Channel1->CNDTR = 0;  
  	DMA1_Channel1->CPAR  = 0;
  	DMA1_Channel1->CMAR = 0;
	DMA1->IFCR |= 0x0F;	             // Reset interrupt pending bits for DMA1 Channel1 
	DMA1->IFCR &= ~0x0F;
 
  	DMA1_Channel1->CPAR = ADC1_DR_Address ; //&(ADC1->DR);
  	DMA1_Channel1->CMAR = (u32)&ADCConvertedValue;
	DMA1_Channel1->CCR  = 0x00003520 ;	//  |= 1<<10 |=1<<8 |=1<<5 |=3<<12;  Circular Perif to MEM ,Half word,Priority high disable  DIR MINC PINC M2M             
  	DMA1_Channel1->CNDTR = 1; 

//DMA使能
	DMA1_Channel1->CCR |=(1<<0);

}
/*******************************************************************************
* Function Name  : Time_Inter_Adjust
* Description    : Adjusts time with interface (time entered by user, using Hyperterminal)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DIS_AD_VAL()
{
         
 	static unsigned long ticks;
	unsigned char Clock1s;
	 while(1)
  	{
		AD_value=ADC1->DR;
   		if (ticks++ >= 9000) 		  /* Set Clock1s to 1 every 1 second    */
		{                  
   		 	ticks   = 0;
    		Clock1s = 1;
  		}
   		/* Printf message with AD value to serial port every 1 second             */
    	if (Clock1s) 
		{
      		Clock1s = 0;
		    USART1_TXword(AD_value);
    	}
  	}
}

void ADC_IRQHandler()
{
   AD_value=ADC1->DR;		 //转换结束标志位EOC在读ADC->DR时清除
   USART1_TXword(AD_value);
}
