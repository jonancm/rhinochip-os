#ifndef MOTORCTL_H
#define MOTORCTL_H

#define T3FREQ                100000                         /* Timer 3 frequency: 100 kHz */
#define T3PERIOD              (1. / T3FREQ)                  /* Timer 3 period: 0.01 ms (0.00001 seconds) */
#define T3PRESCALER           1                              /* Timer 3 prescale value of 1:1 */
#define PR3VAL                (FCY / (T3FREQ * T3PRESCALER)) /* Value for the PR1 register of Timer 3 */

inline void motorctl_setup(void);
inline void motorctl(void);
void motorctl_enable_pid(unsigned char motors);
void motorctl_disable_pid(unsigned char motors);

#endif
