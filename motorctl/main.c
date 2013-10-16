#include <p30fxxxx.h>

// Configuration settings
_FOSC(CSW_FSCM_OFF & FRC_PLL16); // Fosc=16x7.5MHz, Fcy=30MHz
_FWDT(WDT_OFF);                  // Watchdog timer off
_FBORPOR(MCLR_DIS);              // Disable reset pin

#include "../delay.h"
#include "pwm.h"

int main(void)
{
	// Set up port pin RB1 to drive the LED D4
	LATBbits.LATB1 = 0;     // Clear Latch bit for RB1 port pin
	TRISBbits.TRISB1 = 0;   // Set the RB1 pin direction to be an output
	
	pwm_setup();
	pwm_set_pdc1(75);
	
	while (1);
	
	return 0;
}
