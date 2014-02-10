#ifndef CLOCK_H
#define CLOCK_H

#include <p30fxxxx.h>

/*
 * Using the on-board crystal oscillator with a PLL multiplier setting of 16x,
 * an overall instruction cycle frequency of about 29.50 MHz can be achieved.
 */
#define FOSC       7372800              /* On-board crystal frequency (7.3728 MHz) */
#define PLLMODE    16                   /* On-chip PLL setting */
#define FCY        (FOSC * PLLMODE / 4) /* Instruction cycle frequency */

#endif // CLOCK_H
