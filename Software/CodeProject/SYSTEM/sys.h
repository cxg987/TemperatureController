#ifndef __SYS_H
#define __SYS_H
#include <stm32f10x_lib.h>

/**************************************************************
* 位带操作，实现51类似的GPIO控制功能       ??
***************************************************************/

#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr&0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)* ((volatile unsigned long* )(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))

//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011C0C
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x4001200C

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011C08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40012008 

//对单个IO引脚的操作 确保n<=16
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n)  //A口引脚n输出
#define PAin(n)	 BIT_ADDR(GPIOA_IDR_Addr, n)  //A口引脚n输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n)  //B口引脚n输出
#define PBin(n)	 BIT_ADDR(GPIOB_IDR_Addr, n)  //B口引脚n输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n)  //C口引脚n输出
#define PCin(n)	 BIT_ADDR(GPIOC_IDR_Addr, n)  //C口引脚n输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n)  //D口引脚n输出
#define PDin(n)	 BIT_ADDR(GPIOD_IDR_Addr, n)  //D口引脚n输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n)  //E口引脚n输出
#define PEin(n)	 BIT_ADDR(GPIOE_IDR_Addr, n)  //E口引脚n输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n)  //F口引脚n输出
#define PFin(n)	 BIT_ADDR(GPIOF_IDR_Addr, n)  //F口引脚n输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n)  //G口引脚n输出
#define PGin(n)	 BIT_ADDR(GPIOG_IDR_Addr, n)  //G口引脚n输入

/**************************************************************
* Ex_NVIC_Config专用定义
***************************************************************/
#define GPIO_A 0
#define GPIO_B 1
#define GPIO_C 2
#define GPIO_D 3
#define GPIO_E 4
#define GPIO_F 5
#define GPIO_G 6
 
#define FTIR 1	 //下降沿触发
#define RTIR 2	 //上升沿触发

void MYRCC_DeInit(void);
void Stm32ClockInit(u8 PLL);
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset); //设置便宜地址
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);           //设置NVIC分组
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group); //设置中断优先级
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM);           //外部中断配置函数（只对GPIOA~G）

#endif

