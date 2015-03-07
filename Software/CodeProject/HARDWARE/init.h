#ifndef _INIT_H_
#define _INIT_H_

#include <stm32f10x_lib.h>
#include "../SYSTEM/sys.h"
#include "../SYSTEM/delay.h"

void GPIOConfig(void);
void Timer3_Init(u16 arr, u16 psc);
void ADC1_Init(void);
void EXTI8_Init(void);
void PeripheralInit(void);

#endif
