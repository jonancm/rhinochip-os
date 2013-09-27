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

/**
 * Set up the UART2 to be used to communicate with the host PC.
 */
void hostcom_setup(void);

#endif
