#include "mctlcom.h"

#include "../clock.h"

#define T2PRESCALER     64    /* Timer 2 prescale value 1:64 */

bool_t mctlcom_timeout = false;

#include "../debug.h"
#ifndef NDEBUG
#include <stdio.h>
#include <string.h>
#endif

inline void mctlcom_setup(void)
{
	/*****************************************************************
	 * Set up Timer 2 to be used for communication timeout           *
	 *****************************************************************/
	
	// Clear the Timer 2 interrupt flag
	
	IFS0bits.T2IF = 0;
	
	// Enable Timer 2 interrupts
	
	IEC0bits.T2IE = 1;
	
	// Set Timer 2 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
	
	T2CONbits.TCKPS = 2;
}

inline void mctlcom_start_timer(unsigned int timeout)
{
	// Set Timer 2 period
	PR2 = (((timeout / 1000.0) * FCY) / T2PRESCALER);
	
	#ifndef NDEBUG
	char buf[64];
	snprintf(buf, 64, "PR2 = %u\n", PR2);
	dbgmsg_uart2(buf, strlen(buf));
	#endif
	
	// Start Timer 2
	T2CONbits.TON = 1;
}

inline void mctlcom_stop_timer(void)
{
	// Stop Timer 2
	T2CONbits.TON = 0;
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
	mctlcom_timeout = true;
	// Clear interrupt flag
	IFS0bits.T2IF = 0;
	
	#ifndef NDEBUG
	dbgmsg_uart2("_T2Interrupt\n");
	#endif
}

int mctlcom_get_response(char *response, int size, unsigned int *timeout)
{
	bool_t    full;
	int       copied = 0;
	
	// If timeout was requested, start timer
	if (*timeout != 0)
	{
		#ifndef NDEBUG
		dbgmsg_uart2("start timer\n");
		#endif
		
		mctlcom_start_timer(*timeout);
	}
	
	// Wait until the receive buffer has at least one full command or a timeout occurs
	while (!mcuicom_cmd_available() && !mctlcom_timeout);
	
	// Stop the timer
	mctlcom_stop_timer();
	
	// If a timeout occurred, reset the timeout flag and stop the timer
	if (mctlcom_timeout)
	{
		mctlcom_timeout = false;
		
		#ifndef NDEBUG
		dbgmsg_uart2("timeout occurred\n");
		#endif
	}
	// Otherwise, if a command has been fully received, get it and extract the response code
	// in order to return it to the callee
	else
	{
		copied = mcuicom_read_cmd(response, size, &full);
		*timeout = false;
		
		#ifndef NDEBUG
		dbgmsg_uart2("response received\n");
		#endif
	}
	
	return copied;
}
