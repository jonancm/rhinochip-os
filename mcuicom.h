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
 * Opcode definitions for the microcontroller intercommunication commands                            *
 *                                                                                                   *
 * The opcode (operation code) is an 8-bit unsigned integer whose two least-significat bits indicate *
 * the number of parameters that the command requires and, thus, the number of valid parameters that *
 * are transferred.                                                                                  *
 *                                                                                                   *
 * This allows for 64 different commands to be defined, since 6 bits are used to identify the action *
 * of the command and the other 2 bits specify the number of parameters. Hence, the opcode has the   *
 * following format:                                                                                 *
 *                                                                                                   *
 *                    Opcode:                                                                        *
 *                    ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐                              *
 *     Bit number:    │  7  │  6  │  5  │  4  │  3  │  2  │  1  │  0  │                              *
 *                    ├─────┴─────┴─────┴─────┴─────┴─────┼─────┴─────┤                              *
 *     Meaning:       │         Command action            │ Amount of │                              *
 *                    │           identifier              │ params    │                              *
 *                    └───────────────────────────────────┴───────────┘                              *
 *                                                                                                   *
 *****************************************************************************************************/

/**
 * Commands to read the encoder position for each motor.
 * This command is sent from the GPMCU to the MCMCU.
 * 
 * This command requires no parameters.
 * 
 * This command returns a single integer value: the encoder position of the motor (-32768 <= x <= 32767)
 */
#define MCUICOM_READ_ENC_MA       0x01 /* Command: read encoder position of motor A */
#define MCUICOM_READ_ENC_MB       0x05 /* Command: read encoder position of motor B */
#define MCUICOM_READ_ENC_MC       0x09 /* Command: read encoder position of motor C */
#define MCUICOM_READ_ENC_MD       0x0D /* Command: read encoder position of motor D */
#define MCUICOM_READ_ENC_ME       0x11 /* Command: read encoder position of motor E */
#define MCUICOM_READ_ENC_MF       0x15 /* Command: read encoder position of motor F */

/**
 * Command to set the destination position for the each motor (measured in encoder steps).
 * This command is sent from the GPMCU to the MCMCU.
 * 
 * This command requires the destination position (integer) to be provided as a parameter (-32768 <= x <= 32767).
 * 
 * This command doesn't yield any response.
 */
#define MCUICOM_SET_DST_MA        0x19 /* Command: set destination position of motor A in encoder steps */
#define MCUICOM_SET_DST_MB        0x1D /* Command: set destination position of motor B in encoder steps */
#define MCUICOM_SET_DST_MC        0x21 /* Command: set destination position of motor C in encoder steps */
#define MCUICOM_SET_DST_MD        0x25 /* Command: set destination position of motor D in encoder steps */
#define MCUICOM_SET_DST_ME        0x29 /* Command: set destination position of motor E in encoder steps */
#define MCUICOM_SET_DST_MF        0x2D /* Command: set destination position of motor F in encoder steps */

/**
 * Command to notify a motor stall.
 * This command is sent from the MCMCU to the GPMCU.
 * 
 * This command requires no parameters.
 * 
 * This command doesn't yield any response.
 */
#define MCUICOM_STALL_NOTIF_MA    0x04 /* Command: stall notification for motor A */
#define MCUICOM_STALL_NOTIF_MB    0x08 /* Command: stall notification for motor B */
#define MCUICOM_STALL_NOTIF_MC    0x0C /* Command: stall notification for motor C */
#define MCUICOM_STALL_NOTIF_MD    0x10 /* Command: stall notification for motor D */
#define MCUICOM_STALL_NOTIF_ME    0x14 /* Command: stall notification for motor E */
#define MCUICOM_STALL_NOTIF_MF    0x18 /* Command: stall notification for motor F */

/**
 * Command to tell a given motor to stop.
 * This command is sent from the GPMCU to the MCMCU.
 * 
 * This command requires no parameters.
 * 
 * This command doesn't yield any response.
 */
#define MCUICOM_STOP_MA           0x1C /* Command: stop motor A (no matter what the position is) */
#define MCUICOM_STOP_MB           0x20 /* Command: stop motor B (no matter what the position is) */
#define MCUICOM_STOP_MC           0x24 /* Command: stop motor C (no matter what the position is) */
#define MCUICOM_STOP_MD           0x28 /* Command: stop motor D (no matter what the position is) */
#define MCUICOM_STOP_ME           0x2C /* Command: stop motor E (no matter what the position is) */
#define MCUICOM_STOP_MF           0x30 /* Command: stop motor F (no matter what the position is) */

/**
 * Command to send a response to the other MCU.
 * This command is sent from the MCMCU to the GPMCU.
 * 
 * This command require the response code (integer) to be provided as a parameter.
 * 
 * This command doesn't yield any response.
 */
#define MCUICOM_RESPONSE          0x31

/*****************************************************************************************************
 * Data structure definitions                                                                        *
 *****************************************************************************************************/

typedef union {
	struct {
		unsigned char opcode;
		int param[3];
	};
	
	/**
	 * Structure to access the union byte-wise.
	 */
	struct {
		unsigned char data[13];
		// sizeof(unsigned char) = 1 byte
		// sizeof(int) = 4 bytes
		// (1 unsigned char x 1 byte) + (3 int x 4 bytes) = 13 bytes
	};
} mcuicom_cmd;

/*****************************************************************************************************
 * Variable definitions                                                                              *
 *****************************************************************************************************/

#include "datastruc/buffer.h"

/**
 * Receive buffer for the communication with the other MCU (using UART1).
 */
extern buffer_t mcuicom_rcv_buf;

/**
 * Transmit buffer for the communication with the other MCU (using UART1).
 */
extern buffer_t mcuicom_xfr_buf;

/*****************************************************************************************************
 * Function declarations                                                                             *
 *****************************************************************************************************/

/**
 * Set up UART1 for intercommunication with the other MCU (motorctl program).
 */
inline void mcuicom_setup(void);

/**
 * Send a command to the other MCU using the UART1.
 * 
 * @param cmd    command to be sent
 *
 * @returns      amount of bytes sent
 */
int mcuicom_send(mcuicom_cmd cmd);

/**
 * Get the total size of the parameters that a given command (opcode) requires.
 * 
 * @param cmd    command whose parameter size is requested
 *
 * @returns      size of the parameters required by the command
 */
short int mcuicom_param_size(mcuicom_cmd *cmd);

#endif
