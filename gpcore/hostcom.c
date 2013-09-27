#include "hostcom.h"

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
