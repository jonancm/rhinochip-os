#ifndef SERIAL_H
#define SERIAL_H

#include <p30fxxxx.h>
#include "clock.h"

#define BAUDRATE        9600                    //Desired Baud Rate
#define BRGVAL          ((FCY/BAUDRATE)/16)-1   //Formula for U1BRG register
                                                //from dsPIC30F Family
                                                //Reference Manual

#ifndef STRLEN
#define STRLEN(s)    (sizeof(s) - 1)
#else
#error "STRLEN already defined somewhere else"
#endif

//UART_Init() sets up the UART for a 8-bit data, No Parity, 1 Stop bit
//at 9600 baud with transmitter interrupts enabled
void serial_setup(void);

/**
 * Write to the transfer buffer data to be transferred.
 * 
 * @param data       Data to be transferred.
 * @param size       Size of the buffer.
 * 
 * @returns          Amount of bytes actually written.
 */
int serial_send(const char * const data, const int size);

#endif // SERIAL_H
