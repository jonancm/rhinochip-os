#ifndef QEI_H
#define QEI_H

#include <p30fxxxx.h>

#include "../types.h"

#define QEA_MA    PORTDbits.RD2
#define QEB_MA    PORTDbits.RD3
#define QEA_MB    PORTEbits.RE0
#define QEB_MB    PORTEbits.RE1
#define QEA_MC    PORTEbits.RE2
#define QEB_MC    PORTEbits.RE3
#define QEA_MD    PORTEbits.RE4
#define QEB_MD    PORTEbits.RE5
#define QEA_ME    PORTFbits.RF0
#define QEB_ME    PORTFbits.RF1
#define QEA_MF    PORTFbits.RF4
#define QEB_MF    PORTFbits.RF5

#define NUM_MOTORS    6

#define MOTOR_A    0
#define MOTOR_B    1
#define MOTOR_C    2
#define MOTOR_D    3
#define MOTOR_E    4
#define MOTOR_F    5

#define MOTOR_A_MAX_RANGE     300
#define MOTOR_A_MIN_RANGE    -300
#define MOTOR_B_MAX_RANGE     300
#define MOTOR_B_MIN_RANGE    -300
#define MOTOR_C_MAX_RANGE     300
#define MOTOR_C_MIN_RANGE    -300
#define MOTOR_D_MAX_RANGE     300
#define MOTOR_D_MIN_RANGE    -300
#define MOTOR_E_MAX_RANGE     300
#define MOTOR_E_MIN_RANGE    -300
#define MOTOR_F_MAX_RANGE     300
#define MOTOR_F_MIN_RANGE    -300

/**
 * 16-bit count registers to count motor steps. Being 16-bit wide, they can
 * store a value between -32768 an 32767.
 */
static int motor_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

/**
 * Flag that indicates whether a motor is stalled.
 */
static bool_t motor_stalled[NUM_MOTORS] = {false, false, false, false, false, false};

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
