#ifndef HARDHOME_H
#define HARDHOME_H

#include <p30fxxxx.h>

/**
 * Input pins where the state of the limit switches can be read.
 * They are multiplexed with the QEA inputs, thus, they use the
 * sime pin as the QEA lines.
 */
#define LMT_MA                 PORTBbits.RB0
#define LMT_MB                 PORTBbits.RB2
#define LMT_MC                 PORTBbits.RB4
#define LMT_MD                 PORTDbits.RD2
#define LMT_ME                 PORTFbits.RF0
#define LMT_MF                 PORTFbits.RF4

#define LMT_SEL                LATFbits.LATF6
#define SEL_QEA                0
#define SEL_LMT                1

#define HARDHOME_PWM_LEVEL1    65
#define HARDHOME_PWM_LEVEL2    25

inline void hardhome_setup(void);
inline void hardhome(void);

#endif
