#ifndef CLOCK_H
#define CLOCK_H

#include <p30fxxxx.h>

/*
 * Using the on-chip Fast RC Oscillator with a nominal frequency of 8 MHz and
 * a PLL multiplier setting of 16x, an overall instruction cycle frequency of
 * 32 MHz can be achieved.
 */
#define FOSC       8000000              /* On-chip Fast RC Oscillator frequency (7.5 MHz) */
#define PLLMODE    16                   /* On-chip PLL setting */
#define FCY        (FOSC * PLLMODE / 4) /* Instruction cycle frequency */

#endif // CLOCK_H
