#ifndef __USART_H
#define __USART_H
#include <stm32f10x_lib.h>
#include "stdio.h"
#include "../SYSTEM/sys.h"
#include "../SYSTEM/delay.h" 


/***********************************************************
* 串口1 调试函数
* 包括串口初始化函数，和中断函数
************************************************************/

u16  CalCrc( u8* msg, u8 N );      //求一字符数组的CRC码
void Usart1_Init(u32 pclk2, u32 bound); 	   
void USART1_TXByte(u8 data);		 //TX 8位数据
void USART1_TXWord(u16  data);		 //TX 16位数据

void USART1_SendString(u8*string, u16 len);
void USART1_Print(u8*str);
void Handle(void);
void ComunicatProcess(void);
#endif

