#include "mctlcom.h"

#include <string.h> // memcpy

#include "../clock.h"

#define T2PRESCALER     64    /* Timer 1 prescale value 1:64 */

bool_t mctlcom_timeout = false;

// debug
#include "../macros.h"
#include "hostcom.h"

/*
void mctlcom_setup(void)
{
	*/
	/*****************************************************************
	 * Set up Timer 2 to be used for communication timeout           *
	 *****************************************************************/
	/*
	// Clear the Timer 2 interrupt flag
	
	IFS0bits.T2IF = 0;
	
	// Enable Timer 2 interrupts
	
	IEC0bits.T2IE = 1;
	
	// Set Timer 2 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
	
	T2CONbits.TCKPS = 1;
	
	// Set Timer 2 interrupt priority to 1 (default: 2)
	
	IPC1bits.T2IP = 1;
}

void mctlcom_start_timer(int timeout)
{
	// Set Timer 2 period
	PR2 = ((timeout * FCY) / T2PRESCALER);
	
	// Start Timer 2
	T2CONbits.TON = 1;
}

void mctlcom_stop_timer(void)
{
	// Stop Timer 2
	T2CONbits.TON = 0;
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
	mctlcom_timeout = true;
	// Clear interrupt flag
	IFS0bits.T2IF = 0;
}
*/

int mctlcom_get_response(char *response, int size, unsigned int *timeout)
{
	const int cmd_buf_size = 128;
	char      cmd[cmd_buf_size];
	bool_t    full;
	int       copied, retcode = 0;
	
	// If timeout was requested, start timer
	/*
	if (*timeout != 0)
		mctlcom_start_timer(*timeout);
	*/
	
	// Wait until the receive buffer has at least one full command or a timeout occurs
	while (!mcuicom_cmd_available() && !mctlcom_timeout);
	copied = mcuicom_read_cmd(response, size, &full);
	
	// If a timeout occurred, reset the timeout flag and stop the timer
	if (mctlcom_timeout)
	{
		mctlcom_timeout = false;
		//mctlcom_stop_timer();
	}
	// Otherwise, if a command has been fully received, get it and extract the response code
	// in order to return it to the callee
	else
	{
		memcpy(response, cmd, copied);
		*timeout = false;
	}
	
	return retcode;
}
