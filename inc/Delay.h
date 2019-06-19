#ifndef DELAY_H_
#define DELAY_H_

#include "stm32f4xx.h"

void delay_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif
