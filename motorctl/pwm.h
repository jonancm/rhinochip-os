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

#define PWM1    LATEbits.LATE0
#define PWM2    LATEbits.LATE2
#define PWM3    LATEbits.LATE4
#define PWM4    LATEbits.LATE1
#define PWM5    LATEbits.LATE3
#define PWM6    LATEbits.LATE5

#define DIR1    LATBbits.LATB6
#define DIR2    LATBbits.LATB7
#define DIR3    LATBbits.LATB8
#define DIR4    LATCbits.LATC13
#define DIR5    LATCbits.LATC14
#define DIR6    LATEbits.LATE8

void pwm_setup(void);
void pwm_set_duty1(int duty);
void pwm_set_duty2(int duty);
void pwm_set_duty3(int duty);
inline void pwm_set_duty4(int duty);
inline void pwm_set_duty5(int duty);
inline void pwm_set_duty6(int duty);
void __attribute__((__interrupt__)) _T1Interrupt(void);

#endif
