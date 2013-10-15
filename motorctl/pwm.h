#ifndef PWM_H
#define PWM_H

#include "clock.h"

#define TPWM            0.020                             /* PWM period: 20 ms */
#define FPWM            (1 / TPWM)
#define PWMPRESCALER    16                                /* PWM prescaler 1:16 */
#define PWMPER          (FCY / (FPWM * PWMPRESCALER) - 1) /* Value for PTPER */

#define PWM1    LATBbits.LATB0
#define PWM2    LATBbits.LATB1
#define PWM3    LATBbits.LATB2
#define PWM4    LATBbits.LATB3
#define PWM5    LATBbits.LATB4
#define PWM6    LATBbits.LATB5

void pwm_setup(void);
void pwm_set_pdc1(int duty);

#endif
