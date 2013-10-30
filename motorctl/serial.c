#include "serial.h"

//UART_Init() sets up the UART for a 8-bit data, No Parity, 1 Stop bit
//at 9600 baud with transmitter interrupts enabled
void serial_setup(void)
{
    U1MODE = 0x0000;        //Clear UART1 registers
    U1STA = 0x0000;

	//Since the SPI1 (SCK1, SDO1, SDI1) pins are multiplexed with the UART
	//(U1TX and U1RX ) pins on this device this demonstration program
	//will use alternate UART1 pins (U1ATX and U1ARX)
	//The SPI1 module is used to comunicate to the LCD Controller while
	//UART1 module is used to communicate with the RS232 port on the PC
    //U1MODEbits.ALTIO = 1;   //Enable U1ATX and U1ARX instead of
                            //U1TX and U1RX pins
    U1MODEbits.UARTEN = 1;  //Enable UART1 module
    U1BRG = BRGVAL;         //Load UART1 Baud Rate Generator

    IFS0bits.U1RXIF = 0;    //Clear UART1 Receiver Interrupt Flag
    IFS0bits.U1TXIF = 0;    //Clear UART1 Transmitter Interrupt Flag
    IEC0bits.U1RXIE = 1;    //Enable UART1 Receiver ISR
    IEC0bits.U1TXIE = 0;    //Disable UART1 Transmitter ISR
    U1STAbits.URXISEL = 3;  //Setup UART1 receiver to interrupt
	                        //when a character is transferred to the
	                        //transmit buffer and as result,
	                        //the transmit buffer becomes full.
    
    U1STAbits.UTXEN = 1;    //Enable UART1 transmitter
}

int serial_send(const char * const data, const int size)
{
	int sent;
	for (sent = 0; sent < size; ++sent)
	{
		while (U1STAbits.UTXBF); // Wait until the transmit buffer is not full
		U1TXREG = data[sent];
	}
	return sent;
}
