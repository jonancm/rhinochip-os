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

#define LMT_ON                 1
#define LMT_OFF                0
#define LMT_MSK                0x00FC            /* Limit Switch Mask to extract the appropriate bits from port B */

#define LMT_MA_MSK             1                 /* Mask to extract the bit corresponding to LMT_MA from port B */
#define LMT_MB_MSK             2                 /* Mask to extract the bit corresponding to LMT_MB from port B */
#define LMT_MC_MSK             4                 /* Mask to extract the bit corresponding to LMT_MC from port B */
#define LMT_MD_MSK             8                 /* Mask to extract the bit corresponding to LMT_MD from port B */
#define LMT_ME_MSK             16                /* Mask to extract the bit corresponding to LMT_ME from port B */
#define LMT_MF_MSK             32                /* Mask to extract the bit corresponding to LMT_MF from port B */

#define LMT_SEL                LATFbits.LATF6    /* Selection line for the QEI-LMT multiplexer */
#define SEL_QEA                0                 /* Select QEA line */
#define SEL_LMT                1                 /* Select LMT line */

#define HARDHOME_PWM_LEVEL1    65                /* PWM level (i.e. duty cycle) for the 1st phase of the hard home process (faster) */
#define HARDHOME_PWM_LEVEL2    25                /* PWM level (i.e. duty cycle) for the 2nd phase of the hard home process (slower) */

inline void hardhome_setup(void);
inline void hardhome(void);

#endif
