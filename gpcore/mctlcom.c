#include "mctlcom.h"

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

int mctlcom_get_response(unsigned int *timeout)
{
	mcuicom_cmd    cmd;
	int            i, j;
	const int      recvd = 5;
	
	/*
	while (!mcuicom_rcv_buf.used);
	cmd.opcode = mcuicom_rcv_buf.data[0];
	
	int recvd, cmd_size = sizeof(cmd.opcode) + mcuicom_param_size(&cmd);
	// Wait until all of the bytes of the command have been received
	while (mcuicom_rcv_buf.used < cmd_size);
	// Read the command from the buffer
	for (recvd = 1; recvd < cmd_size; ++recvd)
		cmd.data[recvd] = mcuicom_rcv_buf.data[recvd];
	
	// Disable UART 1 receive interrupts to prevent CPU from messing
	// with the 'mcuicom_rcv_buf' buffer
	IEC0bits.U1RXIE = 0;
	
	int i, j;
	// Shift data in the buffer to remove consumed command
	for (i = 0, j = recvd; j < mcuicom_rcv_buf.used; ++i, ++j)
		mcuicom_rcv_buf.data[i] = mcuicom_rcv_buf.data[j];
	mcuicom_rcv_buf.used -= recvd;
	
	// Re-enable UART 1 receive interrupts
	IEC0bits.U1RXIE = 1;
	*/
	
	// If timeout was requested, start timer
	/*
	if (*timeout != 0)
		mctlcom_start_timer(*timeout);
	*/
	
	hostcom_send("before\n", STRLEN("before\n")); // debug
	// Wait until the receive buffer has at least one full command or a timeout occurs
	while (mcuicom_rcv_buf.used < recvd && !mctlcom_timeout);
	hostcom_send("after\n", STRLEN("after\n")); // debug
	
	// If timeout was requested, stop timer
	/*
	if (*timeout != 0)
		mctlcom_stop_timer();
	*/
	
	// If a timeout occurred, reset the timeout flag and don't do anything else
	if (mctlcom_timeout)
		mctlcom_timeout = false;
	// Otherwise, if a command has been fully received, get it and extract the response code
	// in order to return it to the callee
	else
	{
		cmd.data[0] = mcuicom_rcv_buf.data[0]; // opcode
		cmd.data[1] = mcuicom_rcv_buf.data[1]; // param 0 (byte 3)
		cmd.data[2] = mcuicom_rcv_buf.data[2]; // param 0 (byte 2)
		cmd.data[3] = mcuicom_rcv_buf.data[3]; // param 0 (byte 1)
		cmd.data[4] = mcuicom_rcv_buf.data[4]; // param 0 (byte 0)
		/*
		TODO:
		cmd = *mcuicom_rcv_buf.data;
		*/
		hostcom_send(cmd.data, 5);
		
		// Disable UART 1 receive interrupts to prevent CPU from messing
		// with the 'mcuicom_rcv_buf' buffer
		IEC0bits.U1RXIE = 0;
		// Shift data in the buffer to remove consumed command
		for (i = 0, j = recvd; j < mcuicom_rcv_buf.used; ++i, ++j)
			mcuicom_rcv_buf.data[i] = mcuicom_rcv_buf.data[j];
		mcuicom_rcv_buf.used -= recvd;
		// Re-enable UART 1 receive interrupts
		IEC0bits.U1RXIE = 1;
		
		*timeout = false;
	}
	
	return cmd.param[0];
}
