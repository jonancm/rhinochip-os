#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H

#include "../types.h"

#define NUM_MOTORS    6

#define MOTOR_A       0
#define MOTOR_B       1
#define MOTOR_C       2
#define MOTOR_D       3
#define MOTOR_E       4
#define MOTOR_F       5

#ifndef MOTOR_STATUS_C

extern int motor_steps[NUM_MOTORS];
extern bool_t motor_stalled[NUM_MOTORS];

#endif /* MOTOR_STATUS_C */

#endif /* MOTOR_STATUS_H */
