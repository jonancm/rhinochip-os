#include <p30fxxxx.h>

// Configuration settings
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "../delay.h"
#include "pwm.h"
#include "qei.h"

// debug
#include <stdio.h>
#include <string.h>
#include "serial.h"

int curr_state = 0;
int prev_state = 0;
//int steps = 0;
long int steps = 0;

int main(void)
{
	int count = 0; // debug
	
	qei_setup();
	
	// debug
	serial_setup();
	
	while (1)
	{
		char buf[64];
		int i;
		
		/*
		snprintf(buf, 64, "%d => %d, %d step(s)\n", prev_state, curr_state, steps);
		for (i = 0; buf[i] != 0; ++i)
		{
			while (U1STAbits.UTXBF); // Wait until the transmit buffer is not full
			U1TXREG = buf[i];
		}
		*/
		
		if (steps < 1000000)
			++steps;
		else
		{
			++count;
			snprintf(buf, 64, "%d\n", count);
			steps = 0;
			
			for (i = 0; buf[i] != 0; ++i)
			{
				while (U1STAbits.UTXBF); // Wait until the transmit buffer is not full
				U1TXREG = buf[i];
			}
		}
	}
	
	return 0;
}
