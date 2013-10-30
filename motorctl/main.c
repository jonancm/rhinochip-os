#include <p30fxxxx.h>

// Configuration settings
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "../delay.h"
#include "pwm.h"
#include "qei.h"

int main(void)
{
	qei_setup();
	
	while (1)
	{
	}
	
	return 0;
}
