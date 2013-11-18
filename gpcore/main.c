#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4013
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#define NDEBUG

#include "hostcom.h"
#include "../types.h"
#include "../macros.h"
#include "shell.h"
#include "../mcuicom.h"

int main(void)
{
	hostcom_setup();
	mcuicom_setup();
	
	// Code for debugging. Send a message over RS232 notifying that the UART 1
	// and the UART 2 of the GPMCU are ready and working fine.
	#ifndef NDEBUG
	{
		char *c = "UART 1 GPMCU ready\n";
		for (; *c != 0; ++c)
		{
			while (U1STAbits.UTXBF);
			U1TXREG = *c;
		}	
	}
	hostcom_send("UART 2 GPMCU ready\n", STRLEN("UART 2 GPMCU ready\n"));
	#endif
	
	// Start shell in interactive mode
	shell_run_interactive();
	
	return 0;
}
