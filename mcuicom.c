#ifndef MCUICOM_C
#define MCUICOM_C
#endif

#include "mcuicom.h"

buffer_t    mcuicom_rcv_buf;
buffer_t    mcuicom_xfr_buf;
char       *xfr_buf_ptr = mcuicom_xfr_buf.data;

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
	// Enable UART1 transmitter ISR
	IEC0bits.U1TXIE = 1;
	
	// Set up UART1 receiver to interrupt when one character is received
	U1STAbits.URXISEL = 0;
	
	// Enable UART1 transmitter
	U1STAbits.UTXEN = 1;
}

int mcuicom_send(mcuicom_cmd cmd)
{
	int buf_pos, sent, cmd_size = sizeof(cmd.opcode) + mcuicom_param_size(&cmd);
	
	// Disable UART1 transmitter interrupts, to prevent CPU from interrupting
	// and modifying the transmit buffer.
	IEC0bits.U1TXIE = 0;
	
	// Write data to the buffer only if the buffer has enough free space
	if (mcuicom_xfr_buf.size - mcuicom_xfr_buf.used >= cmd_size)
	{
		// Start appending data after the last used position of the buffer
		for (buf_pos = mcuicom_xfr_buf.used, sent = 0; sent < cmd_size; ++sent, ++buf_pos)
			mcuicom_xfr_buf.data[buf_pos] = cmd.data[sent];
		
		// If data has been written to the buffer and no transfer is in progress, start a new transfer
		if (sent && U1STAbits.TRMT)
		{
			xfr_buf_ptr = &mcuicom_xfr_buf.data[mcuicom_xfr_buf.used];
			U1TXREG = *xfr_buf_ptr;
			++xfr_buf_ptr;
		}
		
		// Update the usage count of the buffer
		mcuicom_xfr_buf.used = buf_pos;
	}
	
	// Re-enable UART1 transmitter interrupts
	IEC0bits.U1TXIE = 1;
	
	return sent;
}

short int mcuicom_param_size(mcuicom_cmd *cmd)
{
	// Extract the two least-significant bits, which are the ones that contain the information
	// on how many parameters the command requires, and multiply that number by the size that
	// each parameter spans in memory.
	return (cmd->opcode & 2) * sizeof(cmd->param[0]);
}

/**
 * UART1 transmit ISR.
 */
void __attribute__((interrupt, auto_psv)) _U1TXInterrupt(void)
{
	char *data_limit = &mcuicom_xfr_buf.data[mcuicom_xfr_buf.used];
	
	// Disable UART1 transmitter interrupts, to prevent CPU from interrupting
	// and modifying the transmit buffer.
	IEC0bits.U1TXIE = 0;
	
	// Write as many bytes to the UART1 transmit buffer as possible, until
	// either the UART1 transmit buffer is full or the end of the 'mcuicom'
	// transmit is reached.
	while (!U1STAbits.UTXBF && xfr_buf_ptr < data_limit)
	{
		U1TXREG = *xfr_buf_ptr;
		++xfr_buf_ptr;
	}
	
	// If the end of the 'mcuicom' transmit buffer has been reached, reset it
	// (i.e. mark as empty and restore pointer to the beginning)
	if (xfr_buf_ptr == &mcuicom_xfr_buf.data[mcuicom_xfr_buf.used])
	{
		xfr_buf_ptr = mcuicom_xfr_buf.data;
		mcuicom_xfr_buf.used = 0;
	}
	
	// Re-enable UART1 transmitter interrupts
	IEC0bits.U1TXIE = 1;
	
	// Clear the UART1 transmitter interrupt flag
	IFS0bits.U1TXIF = 0;
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
