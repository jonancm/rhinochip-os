#ifndef PWM_H
#define PWM_H

#include "clock.h"

#define TPWM            0.020                             /* PWM period: 20 ms */
#define FPWM            (1 / TPWM)
#define PWMPRESCALER    1                                 /* PWM prescaler 1:1 */
#define PWMPER          (FCY / (FPWM * PWMPRESCALER) - 1) /* Value for PTPER */

#define PWM1L    LATEbits.LATE0
#define PWM1H    LATEbits.LATE1
#define PWM2L    LATEbits.LATE2
#define PWM2H    LATEbits.LATE3
#define PWM3L    LATEbits.LATE4
#define PWM3H    LATEbits.LATE5

void pwm_setup(void);
void pwm_generate_pdc1(int duty);

#endif
