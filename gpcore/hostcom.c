#include "hostcom.h"

#include "../datastruc/buffer.h"
#include "../hostcmdset.h"

/**
 * Receive buffer for the communication with the host PC (using UART2).
 */
static buffer_t hostcom_rcv_buf;

static short int first_cmdend = -1;

void hostcom_setup(void)
{
	// Set up the receive buffer
	buffer_init(&hostcom_rcv_buf);
	
	// Clear UART2 registers
	U2MODE = 0x0000;
	U2STA = 0x0000;
	
	// Enable UART2 module
	U2MODEbits.UARTEN = 1;
	
	// Set up UART2 baud rate
	U2BRG = BRGVAL;
	
	// Clear UART2 receiver interrupt flag
	IFS1bits.U2RXIF = 0;
	// Clear UART2 transmitter interrupt flag
	IFS1bits.U2TXIF = 0;
	
	// Enable UART2 receiver ISR
	IEC1bits.U2RXIE = 1;
	// Disable UART2 transmitter ISR
	IEC1bits.U2TXIE = 0;
	
	// Set up UART2 receiver to interrupt when a character is transferred to the
	// transmit buffer and, as a result, the transmit buffer becomes full.
	U2STAbits.URXISEL = 3;
	
	// Enable UART2 transmitter
	U2STAbits.UTXEN = 1;
}

void __attribute__((__interrupt__)) _U2RXInterrupt(void)
{
	int i = 0;
	
	while (hostcom_rcv_buf.used < hostcom_rcv_buf.size && i++ < 4)
	{
		// Read the received byte from the UART2 receive register
		hostcom_rcv_buf.data[hostcom_rcv_buf.used] = U2RXREG;
		
		// If this byte is a command separator and no command separator has been
		// found previously, remember its position in the buffer
		if (first_cmdend < 0 &&
		    hostcom_rcv_buf.data[hostcom_rcv_buf.used] == *CMDEND)
		{
			first_cmdend = hostcom_rcv_buf.used;
		}
		
		// Increment the count of elements stored in the buffer
		++hostcom_rcv_buf.used;
	}
	
	// Clear the UART2 receiver interrupt flag
	IFS1bits.U2RXIF = 0;
}

size_t hostcom_read_cmd(byte_t buf[], size_t size, bool_t *full)
{
	size_t copied;
	size_t i, j;
	
	// Copy received data to the user's buffer
	for (copied = 0; copied <= first_cmdend && copied < size; ++copied)
		buf[copied] = hostcom_rcv_buf.data[copied];
	
	// Set the full flag accordingly
	if (copied < first_cmdend)
		*full = false;
	else
		*full = true;
	
	// Shift the data in the buffer to remove already copied data
	if (copied)
	{
		for (i = 0, j = copied; j < hostcom_rcv_buf.size; ++i, ++j)
			hostcom_rcv_buf.data[i] = hostcom_rcv_buf.data[j];
		hostcom_rcv_buf.used -= copied;
	}
	
	return copied;
}

int hostcom_send(const char * const data, const int size)
{
	int sent;
	for (sent = 0; sent < size; ++sent)
	{
		while (U2STAbits.UTXBF); // Wait while the transmit buffer is full
		U2TXREG = data[sent];
	}
	return sent;
}
