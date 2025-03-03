#include "stm32f30x.h"

#ifndef __DELAY_H__
#define __DELAY_H__

#define delay delay_ms

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

#endif
