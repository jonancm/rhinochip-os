#ifndef MOTORCTL_H
#define MOTORCTL_H

#define T3FREQ                5000                           /* Timer 3 frequency: 5 kHz */
#define T3PERIOD              (1. / T3FREQ)                  /* Timer 3 period: 0.2 ms (0.0002 seconds) */
#define T3PRESCALER           1                              /* Timer 3 prescale value of 1:1 */
#define PR3VAL                (FCY / (T3FREQ * T3PRESCALER)) /* Value for the PR1 register of Timer 3 */

inline void motorctl_setup(void);

/**
 * PID control loop.
 * 
 * This function is called on a timely basis by Timer 3 in order to perform PID
 * control on the motors to compensate any disturbances. It recalculates the PWM
 * duty cycle needed to keep the motors at the desired position while no
 * trapezoidal movement is in course (if the motor is in closed-loop mode).
 */
inline void motorctl(void);

/**
 * Perform a trapezoidal movement from the actual position to the commanded
 * position (stored in the commanded position register by the PD and PR host
 * commands).
 */
void motorctl_move(void);

/**
 * Enable PID control on the specified motors.
 */
void motorctl_enable_pid(unsigned char motors);

/**
 * Disable PID control on the specified motors.
 */
void motorctl_disable_pid(unsigned char motors);

#endif
