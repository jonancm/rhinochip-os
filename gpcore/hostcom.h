/**
 * @file
 * @brief Host communication module
 * 
 * Module to communicate the GPMCU with the host PC over RS-232C, so that the
 * host PC can operate the RhinoChip controller using the host command set.
 */

#ifndef HOSTCOM_H
#define HOSTCOM_H

#include "clock.h"

#define BAUDRATE        9600                    //Desired Baud Rate
#define BRGVAL          ((FCY/BAUDRATE)/16)-1   //Formula for U1BRG register
                                                //from dsPIC30F Family
                                                //Reference Manual

#include "../types.h"

/**
 * Set up the UART2 to be used to communicate with the host PC.
 */
void hostcom_setup(void);

/**
 * UART2 receive ISR.
 */
void __attribute__((__interrupt__)) _U2RXInterrupt(void);

/**
 * Copy the first fully received host command from the receive buffer to the
 * user's buffer. If no host command has been fully received yet, don't copy
 * anything. If a host command has been fully received, it will be copied
 * alongside with the command separator.
 * 
 * @param buf     Buffer to copy the command to.
 * @param size    Size of the buffer.
 * @param full    Output flag set to true if the command has been fully copied
 *                (i.e. the buffer had enough free space) and set to false if
 *                the command could not be fully copied to the buffer (i.e. the
 *                buffer didn't have enough free space).
 * 
 * @returns       Amount of bytes copied.
 */
size_t hostcom_read_cmd(byte_t buf[], size_t size, bool_t *full);

int hostcom_send(const char * const data, const int size);

#endif
