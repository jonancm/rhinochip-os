#ifndef QEI_H
#define QEI_H

#include <p30fxxxx.h>

#define QEA_MA    LATBbits.LATB0
#define QEB_MA    LATBbits.LATB1
#define QEA_MB    LATBbits.LATB2
#define QEB_MB    LATBbits.LATB3
#define QEA_MC    LATBbits.LATB4
#define QEB_MC    LATBbits.LATB5
#define QEA_MD    LATBbits.LATB6
#define QEB_MD    LATBbits.LATB7
#define QEA_ME    LATBbits.LATB8
#define QEB_ME    LATCbits.LATC15
#define QEA_MF    LATDbits.LATD2
#define QEB_MF    LATDbits.LATD3

#define NUM_MOTORS    6

#define MOTOR_A    0
#define MOTOR_B    1
#define MOTOR_C    2
#define MOTOR_D    3
#define MOTOR_E    4
#define MOTOR_F    5


/**
 * Buffer to store the current encoder state for each of the six motors, i.e.,
 * the values of QEA and QEB represented as a 2-bit number (b00, b01, b10, b11).
 */
static char curr_encoder_state[NUM_MOTORS];

/**
 * 16-bit count registers to count motor steps. Being 16-bit wide, they can
 * store a value between -32768 an 32767.
 */
static int motor_steps[NUM_MOTORS];

/**
 * Set up the Quadrature Encoder Interface.
 */
void qei_setup(void);

/**
 * Timer 1 ISR to sample the QEA and QEB lines for each motor and count motor
 * steps.
 */
void __attribute__((__interrupt__)) _T2Interrupt(void);

#endif
