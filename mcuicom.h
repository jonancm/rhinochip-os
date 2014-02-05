#ifndef MCUICOM_H
#define MCUICOM_H

/*****************************************************************************************************
 * Serial port timing configuration: 9600 baud                                                       *
 *****************************************************************************************************/

#include "clock.h"

#define BAUDRATE        9600                    //Desired Baud Rate
#define BRGVAL          ((FCY/BAUDRATE)/16)-1   //Formula for U1BRG register
                                                //from dsPIC30F Family
                                                //Reference Manual

/*****************************************************************************************************
 * Variable definitions                                                                              *
 *****************************************************************************************************/

#include "datastruc/buffer.h"

#ifndef MCUICOM_C

/**
 * Receive buffer for the communication with the other MCU (using UART1).
 */
extern buffer_t mcuicom_rcv_buf;

#endif

/*****************************************************************************************************
 * Function declarations                                                                             *
 *****************************************************************************************************/

/**
 * Set up UART1 for intercommunication with the other MCU (motorctl program).
 */
inline void mcuicom_setup(void);

int mcuicom_send(const char * const data);

/**
 * Copy the first fully received GPMCU command from the receive buffer to the
 * user's buffer. If no GPMCU command has been fully received yet, don't copy
 * anything. If a GPMCU command has been fully received, it will be copied
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
int mcuicom_read_cmd(char buf[], int size, bool_t *full);

/**
 * Return true if a full command is available (i.e. a command end mark has been
 * received).
 */
bool_t mcuicom_cmd_available(void);

#endif
