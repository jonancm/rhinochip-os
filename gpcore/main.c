#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4013
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "hostcom.h"
#include "../types.h"
#include "../macros.h"
#include "shell.h"

#define BUF_SIZE    64

#define LCD_READY    "GPMCU ready"
#define MSG_READY    "GPMCU ready\n"

int main(void)
{
	hostcom_setup();
	
	// Set up port pin RB0 the LED D3
	LATBbits.LATB0 = 0;     // Clear Latch bit for RB0 port pin
	TRISBbits.TRISB0 = 0;   // Set the RB0 pin direction to be an output
	
	hostcom_send(MSG_READY, STRLEN(MSG_READY));
	
	shell_run_interactive();
	
	return 0;
}
