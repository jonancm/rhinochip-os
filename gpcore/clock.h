#ifndef CLOCK_H
#define CLOCK_H

#include <p30fxxxx.h>

//Defines for System Clock Timing -
//For oscillator configuration XT x PLL8 mode,
//Device Throughput in MIPS = Fcy = 7372800*8/4 = ~14.74 MIPS
//Instruction Cycle time = Tcy = 1/(Fcy) = ~68 nanoseconds
#define XTFREQ          7372800         //On-board Crystal frequency
#define PLLMODE         8               //On-chip PLL setting
#define FCY             XTFREQ*PLLMODE/4        //Instruction Cycle Frequency

#endif // CLOCK_H
