#ifndef DELAY_H
#define DELAY_H

#include "../clock.h"

void delay_us(unsigned int duration)
{
	#define NINST 2 // Number of instructions
	unsigned long count, limit = duration / (NINST * FCY);
	for (count = 0; count < limit; ++count);
}

void delay_ms(unsigned int duration)
{
	delay_us(duration * 1000);
}

void delay_s(unsigned int duration)
{
	delay_us(duration * 1000000);
}

#endif
