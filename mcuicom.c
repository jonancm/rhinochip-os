#ifndef MCUICOM_C
#define MCUICOM_C
#endif

#include "mcuicom.h"
#include "hostcmdset.h"
#include "macros.h"

static buffer_t mcuicom_rcv_buf;

static int first_cmdend = -1;

//debug
#include <stdio.h>

inline void mcuicom_setup(void)
{
	// Set up the receive buffer
	buffer_init(&mcuicom_rcv_buf);
	
	// Clear UART1 registers
	U1MODE = 0x0000;
	U1STA = 0x0000;
	
	// Enable UART1 module
	U1MODEbits.UARTEN = 1;
	
	// Set up UART1 baud rate
	U1BRG = BRGVAL;
	
	// Clear UART1 receiver interrupt flag
	IFS0bits.U1RXIF = 0;
	// Clear UART1 transmitter interrupt flag
	IFS0bits.U1TXIF = 0;
	
	// Enable UART1 receiver ISR
	IEC0bits.U1RXIE = 1;
	// Disable UART1 transmitter ISR
	IEC0bits.U1TXIE = 0;
	
	// Set up UART1 receiver to interrupt when one character is received
	U1STAbits.URXISEL = 0;
	
	// Enable UART1 transmitter
	U1STAbits.UTXEN = 1;
}

int mcuicom_send(const char * const data, const int size)
{
	int sent;
	for (sent = 0; sent < size; ++sent)
	{
		while (U1STAbits.UTXBF); // Wait while the transmit buffer is full
		U1TXREG = data[sent];
	}
	return sent;
}

/**
 * UART1 receive ISR.
 */
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
	// Disable UART1 receiver interrupts, to prevent CPU from interrupting
	// and modifying the receive buffer.
	IEC0bits.U1RXIE = 0;
	
	// debug
	/*
	#ifdef __dsPIC30F4011__
	if (U1STAbits.URXDA)
		mcuicom_send("data received\n", STRLEN("data received\n"));
	if (mcuicom_rcv_buf.used < mcuicom_rcv_buf.size)
		mcuicom_send("used < size\n", STRLEN("used < size\n"));
	#endif
	*/
	
	// While UART1 receive buffer has data and the 'mcuicom_rcv_buf' has free
	// space...
	while (U1STAbits.URXDA && mcuicom_rcv_buf.used < mcuicom_rcv_buf.size)
	{
		// Read the received byte from the UART1 receive register
		mcuicom_rcv_buf.data[mcuicom_rcv_buf.used] = U1RXREG;
		
		// debug
		/*
		#ifdef __dsPIC30F4011__
		mcuicom_send(&mcuicom_rcv_buf.data[mcuicom_rcv_buf.used], 1);
		if (mcuicom_rcv_buf.data[mcuicom_rcv_buf.used] == '\r')
			mcuicom_send("carriage return received\n", STRLEN("carriage return received\n"));
		#endif
		*/
		
		// If this byte is a command separator and no command separator has been
		// found previously, remember its position in the buffer
		if (first_cmdend < 0 &&
		    mcuicom_rcv_buf.data[mcuicom_rcv_buf.used] == *CMDEND)
		{
			// debug
			/*
			#ifdef __dsPIC30F4011__
			mcuicom_send("first_cmdend set\n", STRLEN("first_cmdend set\n"));
			#endif
			*/
			
			first_cmdend = mcuicom_rcv_buf.used;
		}
		
		// Increment the count of elements stored in the buffer
		++mcuicom_rcv_buf.used;
	}
	
	// debug
	/*
	#ifdef __dsPIC30F4011__
	mcuicom_send("_U1RXInterrupt\n", STRLEN("_U1RXInterrupt\n"));
	#endif
	*/
	
	// Re-enable UART1 receiver interrupts
	IEC0bits.U1RXIE = 1;
	
	// Clear the UART1 receiver interrupt flag
	IFS0bits.U1RXIF = 0;
}

int mcuicom_read_cmd(char buf[], int size, bool_t *full)
{
	int copied;
	int i, j;
	
	// Copy received data to the user's buffer
	for (copied = 0; copied <= first_cmdend && copied < size; ++copied)
		buf[copied] = mcuicom_rcv_buf.data[copied];
	
	// Set the full flag accordingly
	if (copied > first_cmdend)
		*full = true;
	else
		*full = false;
	
	// Shift the data in the buffer to remove already copied data and search
	// for the next command end marker
	if (copied)
	{
		// Disable UART1 receive interrupt to prevent the ISR from messing
		// around with 'mcuicom_rcv_buf.used'
		IEC0bits.U1RXIE = 0;
		
		for (i = 0, j = copied; j < mcuicom_rcv_buf.used; ++i, ++j)
			mcuicom_rcv_buf.data[i] = mcuicom_rcv_buf.data[j];
		mcuicom_rcv_buf.used -= copied;
		
		for (i = 0;
		     i < mcuicom_rcv_buf.used && mcuicom_rcv_buf.data[i] != *CMDEND;
		   ++i);
		if (i < mcuicom_rcv_buf.used)
			first_cmdend = i;
		else
			first_cmdend = -1;
		
		// Enable UART1 receive interrupt again
		IEC0bits.U1RXIE = 1;
	}
	
	return copied;
}

bool_t mcuicom_cmd_available(void)
{
	return first_cmdend > -1;
}
