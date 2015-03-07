#ifndef _OPTION_H_
#define _OPTION_H_

#define _DEBUG		0

#define BAUDRATE	4800	      //USART1波特率(9600,19200, 38400, 56000, 115200)
#define AD_MIN		0
#define AD_MAX		0x0FFF

#define CHANNEL		16 		  //通道数 
#define SAMP_TIME	10         //每个通道的采样次数
#define WORK_CYCLE	100		  //工作周期

#define PARAM_ADDR	0x0800C000
#define PARAM_NUM	20

#endif
