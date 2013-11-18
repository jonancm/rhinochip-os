#ifndef MCUICOM_C
#define MCUICOM_C
#endif

#include "mcuicom.h"

buffer_t    mcuicom_rcv_buf;
buffer_t    mcuicom_xfr_buf;
char       *xfr_buf_ptr = mcuicom_xfr_buf.data;

//debug
#include <stdio.h>

inline void mcuicom_setup(void)
{
	// Set up the receive buffer
	buffer_init(&mcuicom_rcv_buf);
	// Set up the transmit buffer
	buffer_init(&mcuicom_xfr_buf);
	
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

/*
int mcuicom_send(mcuicom_cmd *cmd)
{
	int sent, cmd_size = sizeof(cmd->opcode) + mcuicom_param_size(cmd);
	for (sent = 0; sent < cmd_size; ++sent)
	{
		while (U1STAbits.UTXBF); // Wait while the transmit buffer is full
		U1TXREG = cmd->data[sent];
	}
	return sent;
}
*/
int mcuicom_send(char msg[])
{
	int sent;
	for (sent = 0; msg[sent] != 0; ++sent)
	{
		while (U1STAbits.UTXBF); // Wait while the transmit buffer is full
		U1TXREG = msg[sent];
	}
	return sent;
}

short int mcuicom_param_size(mcuicom_cmd *cmd)
{
	// Extract the two least-significant bits, which are the ones that contain the information
	// on how many parameters the command requires, and multiply that number by the size that
	// each parameter spans in memory.
	return (cmd->opcode & 3) * sizeof(cmd->param[0]);
}

/**
 * UART1 receive ISR.
 */
void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
	// Disable UART1 receiver interrupts, to prevent CPU from interrupting
	// and modifying the transmit buffer.
	IEC0bits.U1RXIE = 0;
	
	// While UART1 receive buffer has data and the 'mcuicom_rcv_buf' has free
	// space...
	while (U1STAbits.URXDA && mcuicom_rcv_buf.used < mcuicom_rcv_buf.size)
	{
		// Read the received byte from the UART1 receive register
		mcuicom_rcv_buf.data[mcuicom_rcv_buf.used] = U1RXREG;
		
		// Increment the count of elements stored in the buffer
		++mcuicom_rcv_buf.used;
	}
	
	// Re-enable UART1 receiver interrupts
	IEC0bits.U1RXIE = 1;
	
	// Clear the UART1 receiver interrupt flag
	IFS0bits.U1RXIF = 0;
}
