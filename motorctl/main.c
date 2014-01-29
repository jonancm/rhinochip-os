#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4011
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "pwm.h"
#include "qei.h"
#include "../mcuicom.h"
#include "gpcorecom.h"
#include "motorctl.h"
#include "controller_status.h"

#include "../debug.h"

#undef NDEBUG

#ifndef NDEBUG
#include <stdio.h>
#define INTERPRET_CMDS ""
#define CLEAR_DISPLAY "\x1B\x5B" "2J" // Erase the screen: ESC [ Ps J
#define CURSOR_HOME "\x1B\x5BH" // ESC [ H
#define DISPLAY_CMDS ""
#endif

int main(void)
{
	pwm_setup();
	qei_setup();
	mcuicom_setup();
	motorctl_setup();
	controller_status_setup();
	
	// Code for debugging. Send a message over RS232 notifying that the UART 1
	// is ready and working fine.
	dbgmsg_uart1("UART 1 MCMCU ready\n");
	
	#ifndef NDEBUG
	printf(CLEAR_DISPLAY);
	#endif
	
	while (1)
	{
		// TODO: two approaches possible, compare and select the best.
		// 1) Perform both the interpretation of commands and the motor control
		//    loop as equally important tasks inside a loop (i.e. one does not
		//    have greater priority over the other).
		// 2) Perform the interpretation of commands as single task in a loop and
		//    perform motor control on a timely basis using interrupts, so that it
		//    has greater priority over command interpretation (which can actually
		//    be less efficient).
		gpcorecom_interpret_next();
		
		#ifndef NDEBUG
		printf(INTERPRET_CMDS    CURSOR_HOME    "% 6d"    DISPLAY_CMDS,
		       motor_steps[MOTOR_A]);
		#endif
	}
	
	return 0;
}

#define NDEBUG
