#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4013
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "hostcom.h"
#include "../types.h"
#include "../macros.h"
#include "shell.h"

int main(void)
{
	hostcom_setup();
	
	shell_run_interactive();
	
	return 0;
}
