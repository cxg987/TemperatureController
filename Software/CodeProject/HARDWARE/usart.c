#include "usart.h"
#include "..\user\option.h"

extern u16	temp[CHANNEL];	               //外部变量声明
extern u8	curt_cycle;
extern u16	comp_temp;				      //全节补偿温度值0.1C° 
extern u16	each_seg_comp[CHANNEL];	      //各节补偿温度值1C°
extern u16	set_temp;					  //存放设定温度值1C°
extern u8	duty_ratio_shadow[CHANNEL];	  //各节继电器输出占空比（0~WORK_CYCLE）
extern u8	frame_space_time;			  //帧间间隔时间


u8  rx_buf[9];				//接收数组
u8  tx_buf[30]; 			//发送数组
u8  recv_count=0;
u8  send_count=0;	 
/*******************************************************************
* 串口1 调试函数
* 包括串口初始化函数，和中断函数
* 支持适应不同频率下的串口波特率设置，增加了串口接收命令功能
********************************************************************/
void Usart1_Init(u32 pclk2, u32 bound)
{												   
	float temp;
	u16 mantissa;
	u16 fraction;
	temp = (float)(pclk2*1000000)/(bound*16);  //temp=USARTDIV,  Tx / Rx 波特率 ＝ fCK / (16* USARTDIV)
 	mantissa = temp;                      //得到整数部分
	fraction = (temp-mantissa)*16;		  //得到小数部分
	mantissa <<= 4;
	mantissa += fraction;

	RCC->APB2ENR |= 1<<3; 				  //使能PORTB口,
	RCC->APB2ENR |= 1<<14;                //USART1时钟
	RCC->APB2ENR |= 1<<0;				  //AFIO时钟使能

	GPIOB->CRL &=0x000FFFFF;
	GPIOB->CRL |=0x4B100000;             //PB5(485_ctr),PB6(Tx1),PB7(RX1)状态设置
	GPIOB->BRR |= GPIO_Pin_5;			 //set 485_ctr for RX
	AFIO->MAPR |= 0x00000004;			 //USART1 remap


	RCC->APB2RSTR |= 1<<14;	    		  //复位USART1
	RCC->APB2RSTR &= ~(1<<14);			  //停止复位

	USART1->BRR = mantissa;				  //设置波特率
	USART1->CR1 |= (1<<12 | 2<<9 );       //1位停止位，校验使能，偶校验，9个数据位（8+1校验位）
	USART1->CR1 |= (1<<13 | 3<<2 );		  //USART,TE,RE使能,8位数据


    USART1->SR  &= ~(1<<5);				 //清标志位RXNE
//  USART1->CR1 |= 1<<8;				 //PE奇偶校验中断使能
	USART1->CR1 |= 1<<5;				 //接收缓冲区非空RXNEIE中断使能
	MY_NVIC_Init(3,1,USART1_IRQChannel,2);	 //组2最低优先级
}

/*******************************************************************
* 发送函数
* 包括8位数据，16位，字符串数据的发送函数
********************************************************************/
/*发送一个字节的数据 */
void USART1_TXByte(u8 data)
{
    GPIOB->BSRR |= GPIO_Pin_5;			//set 485_ctr for TX

	while (!(USART1->SR&0x80));		//查询SR中的TXE
	USART1->DR = (data & 0xFF);
	while (!(USART1->SR&0x40));		//查询SR中的TC
	while (!(USART1->SR&0x80));		//查询SR中的TXE
	while (!(USART1->SR&0x40));		//查询SR中TC标志

	GPIOB->BRR |= GPIO_Pin_5;
}

/*发送一个16位数据 */
void USART1_TXWord(u16 data)
{	 
	USART1_TXByte(data/256);
	USART1_TXByte(data%256);       //先发送高字节，后发低字节	 
}

/*发送一个字符串   */
void USART1_SendString(u8*string, u16 len)
{
	u16 i;
	for(i=0; i<len; i++)
	{	
		USART1_TXByte(string[i]);
	}
}

void USART1_Print(u8*str)
{
	while(*str != '\0')
	{	
		USART1_TXByte(*str++);
	}
}

/*******************************************************************
* 接收函数
* 标志位检测方式 和 中断接收函数
********************************************************************/

/*接收中断 */
void USART1_IRQHandler(void)
{											
	u8 t;

	t = USART1->DR;		 //对USART_DR 的读操作可将RXNE标志位清零
	frame_space_time = curt_cycle;

	if( recv_count < 8 )	
	{
		rx_buf[recv_count] = t;
		recv_count++;
	}

}

/*******************************************************************
*  功    能： 计算CRC码
*  入口参数:  待求CRC码的数组
*  返    回： framcrc[2]
********************************************************************/
u16 CalCrc( u8* msg, u8 N )
{
	u8 i,j,t;
	u16 crc = 0xFFFF;
	
   for(i=0; i<N; i++)
   {
   		crc ^= (u16)msg[i];
		for(j=0; j<8; j++)
		{
			t = crc & 1;
			crc = crc >> 1;		 //右移一位
			crc = crc & 0x7fff;	 //最高位清零
            if (t == 1)			 //若移出的LSB为1则crc^0xa001 ,若LSB=0，crc不变
            crc = crc ^ 0xa001;
            crc = crc & 0xffff;
		}
   }
   return crc;
} 

/*******************************************************************
*  功    能：接收数据处理，包括CRC校验，命令解析，处理
*  入口参数: 无
*  返    回：无
********************************************************************/
void  Handle()
{	
	u8  reqflag;			//请求处理标志
	u8  except;		        //异常标志（0x01,0x02,0x03,0x04）
	u16 framcrc;		    //帧校验码
	u8  add;				//设备地址
	u8  i;

	framcrc = CalCrc(rx_buf, 6);
	//若校验码正确，解析命令
	if( ( rx_buf[7]<<8 | rx_buf[6] ) == framcrc )		
	{
	    add = ( (~ GPIOA->IDR) >>8 ) & 0x0000000F ;				//读设备地址
		if ( rx_buf[0] == add )									//地址符合
		{
		     reqflag=0;
			 except =0;
			 switch( rx_buf[1] )							   
			 {
			   case 6:	if( rx_buf[2] == 0x20 && rx_buf[3]==0x01 )
			          	{ reqflag = 1; break;}	   //设定温度命令请求
						if( rx_buf[2] == 0x20 && rx_buf[3]==0x02 )
						{ reqflag = 2; break;}	   //设定全节温度补偿请求
						if( rx_buf[2] == 0x20 && rx_buf[3]>=0x10 && rx_buf[3]<=0x1B )
						{ reqflag = 3; break;}	   //设定各节温度补偿请求
						if( rx_buf[2] == 0x21 && rx_buf[3]>=0x10 && rx_buf[3]<=0x1B && rx_buf[5]<=100)
						{ reqflag = 5; break;}	   //设定各节输出占空比请求

			   		   	else 
						except = 0x02;

						break;

               case 3:  if( rx_buf[2] == 0x21 && rx_buf[3]==0x00 )
			            {
							 if( rx_buf[4]==0 && (rx_buf[5]&0x0F) <= 12)
							    reqflag = 4;	   //返回温度命令请求处理
							 else
							    except = 0x03;	   //寄存器数量异常   
						}
			   		   	else 
						except = 0x02;			   //起始地址异常

						break;


			   default:  except = 0x01;			   //功能码异常
			 }
		}
	}
//命令解析完，开始处理请求
	switch( reqflag )
	{
		case 1: //处理设定温度命令
				set_temp = rx_buf[4]<<8 | rx_buf[5];

				tx_buf[0] = add;
		        tx_buf[1] = 0x06;
				tx_buf[2] = 0x20;
				tx_buf[3] = 0x01;
				tx_buf[4] = 0x01;
				tx_buf[5] = 0x73;
//				framcrc   = CalCrc(tx_buf,6);
//				tx_buf[6] = framcrc & 0xff;															
//				tx_buf[7] = framcrc >> 8;
				tx_buf[6] = rx_buf[6];															
				tx_buf[7] = rx_buf[7];
			    USART1_SendString(tx_buf,8);
				break;
											 
		case 2:	 //处理全节温度补偿(1°)
				comp_temp = rx_buf[4]<<8 | rx_buf[5];		        
                tx_buf[0] = add;
		        tx_buf[1] = 0x06;
				tx_buf[2] = 0x20;
				tx_buf[3] = 0x02;
				tx_buf[4] = 0x01;
				tx_buf[5] = 0x73;
//				framcrc   = CalCrc(tx_buf,6);
//				tx_buf[6] = framcrc & 0xff;															
//				tx_buf[7] = framcrc >> 8;
				tx_buf[6] = rx_buf[6];															
				tx_buf[7] = rx_buf[7];
				USART1_SendString(tx_buf, 8);
				break;

		case 3:	 //处理各节温度补偿（0.1C°，50为基准50=0C°）
				i = rx_buf[3]&0x0F;
				each_seg_comp[i] = rx_buf[4]<<8 | rx_buf[5];		

                tx_buf[0] = add;
		        tx_buf[1] = 0x06;
				tx_buf[2] = 0x20;
				tx_buf[3] = rx_buf[3];
				tx_buf[4] = 0x01;
				tx_buf[5] = 0x73;
//				framcrc   = CalCrc(tx_buf,6);
//				tx_buf[6] = framcrc & 0xff;															
//				tx_buf[7] = framcrc >> 8;
				tx_buf[6] = rx_buf[6];															
				tx_buf[7] = rx_buf[7];
				USART1_SendString(tx_buf, 8);
				break;

		case 4:	 //处理返回温度命令
                tx_buf[0] = add;
		        tx_buf[1] = 0x03;
				tx_buf[2] = rx_buf[5]*2;
				for(i=0,send_count=3; i<rx_buf[5]; i++,send_count+=2)
				{
					tx_buf[send_count]   = temp[i]>>8;   //注Temp为字符数组，要发送的温度是16位
					tx_buf[send_count+1] = temp[i]&0xff;
				}
				framcrc = CalCrc(tx_buf,send_count);
				tx_buf[send_count++] = framcrc & 0xff;															
				tx_buf[send_count++] = framcrc >> 8;
			//	tx_buf[6] = rx_buf[6];															
			//	tx_buf[7] = rx_buf[7];
				USART1_SendString(tx_buf, send_count);
				break;

		case 5:	 //处理设定各节输出占空比（0~100）命令
				i = rx_buf[3]&0x0F;
				duty_ratio_shadow[i] = rx_buf[5];		

                tx_buf[0] = add;
		        tx_buf[1] = 0x06;
				tx_buf[2] = 0x21;
				tx_buf[3] = rx_buf[3];
				tx_buf[4] = 0x01;
				tx_buf[5] = 0x73;
//				framcrc   = CalCrc(tx_buf,6);
//				tx_buf[6] = framcrc & 0xff;															
//				tx_buf[7] = framcrc >> 8;
				tx_buf[6] = rx_buf[6];															
				tx_buf[7] = rx_buf[7];
				USART1_SendString(tx_buf, 8);
				break;

		default: break;
	}
        if( except != 0 )
		{
			tx_buf[0] = add;
			tx_buf[1] = rx_buf[1] | 0x80 ;
			tx_buf[2] = except;	
			framcrc   = CalCrc(tx_buf,3);
			tx_buf[3] = framcrc & 0xff;															
			tx_buf[4] = framcrc >> 8;
			USART1_SendString(tx_buf,5);		
		} 		
}

void ComunicatProcess()
{
	 while( recv_count )		//若有收到数据
	{
		 if( ( frame_space_time < curt_cycle) && (curt_cycle - frame_space_time > 10 )	 //判断帧结束
		   ||( frame_space_time > curt_cycle) && (frame_space_time - curt_cycle < 90 ) )
		{
			if( recv_count == 8 )
			{
				Handle();
			}
			recv_count = 0;
		}
	}
}

