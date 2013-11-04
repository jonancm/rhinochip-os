#ifndef PWM_H
#define PWM_H

#include "clock.h"

#define TPWM            0.018                             /* PWM period: 18 ms */
#define FPWM            (1 / TPWM)                        /* PWM frequency: 50 Hz */
#define T1PRESCALER     1                                 /* Timer 1 prescale value 1:1 */
#define PWMRESOL        100                               /* PWM resolution (no. of counts of Timer 1) */
#define T1PERIOD        (TPWM / PWMRESOL)                 /* Period of Timer 1 */
#define PR1VAL          ((T1PERIOD * FCY) / T1PRESCALER)  /* Value for the PR1 register */
#define PWMPRESCALER    64                                /* PWM prescaler 1:64 */
#define PWMPER          (FCY / (FPWM * PWMPRESCALER) - 1) /* Value for the PTPER register */

#define PWM1            LATEbits.LATE0 /* 1st Hardware PWM channel */
#define PWM2            LATEbits.LATE2 /* 2nd Hardware PWM channel */
#define PWM3            LATEbits.LATE4 /* 3rd Hardware PWM channel */
#define PWM4            LATEbits.LATE1 /* 1st Software PWM channel */
#define PWM5            LATEbits.LATE3 /* 2nd Software PWM channel */
#define PWM6            LATEbits.LATE5 /* 3rd Software PWM channel */

#define DIR1            LATBbits.LATB6  /* Direction signal output for PWM channel #1 */
#define DIR2            LATBbits.LATB7  /* Direction signal output for PWM channel #2 */
#define DIR3            LATBbits.LATB8  /* Direction signal output for PWM channel #3 */
#define DIR4            LATCbits.LATC13 /* Direction signal output for PWM channel #4 */
#define DIR5            LATCbits.LATC14 /* Direction signal output for PWM channel #5 */
#define DIR6            LATEbits.LATE8  /* Direction signal output for PWM channel #6 */

void pwm_setup(void);
inline void pwm_set_duty1(int duty);
inline void pwm_set_duty2(int duty);
inline void pwm_set_duty3(int duty);
inline void pwm_set_duty4(int duty);
inline void pwm_set_duty5(int duty);
inline void pwm_set_duty6(int duty);
void __attribute__((__interrupt__)) _T1Interrupt(void);

#endif
