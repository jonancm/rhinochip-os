#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4011
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "pwm.h"
#include "qei.h"
#include "../mcuicom.h"
#include "gpcorecom.h"

#include "../debug.h"

int main(void)
{
	pwm_setup();
	qei_setup();
	mcuicom_setup();
	
	pwm_set_duty1(0);
	pwm_set_duty2(0);
	pwm_set_duty3(0);
	pwm_set_duty4(0);
	pwm_set_duty5(0);
	pwm_set_duty6(0);
	
	// Code for debugging. Send a message over RS232 notifying that the UART 1
	// is ready and working fine.
	dbgmsg_uart1("UART 1 MCMCU ready\n");
	
	while (1)
	{
		// TODO: two approaches possible, compare and select the best.
		// 1) Perform both the interpretation of commands and the motor control
		//    loop as equally important tasks inside a loop (i.e. one does not
		//    have greater priority over the other).
		// 2) Perform the interpretation of commands as single task in a loop and
		//    perform motor control on a timely basis using interrupts, so that it
		//    has greater priority over command interpretation (which can actually
		//    be less efficiente).
		gpcorecom_interpret_next();
	}
	
	return 0;
}
