#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H

#include "../types.h"

#define NUM_MOTORS     6

#define MOTOR_A        0
#define MOTOR_B        1
#define MOTOR_C        2
#define MOTOR_D        3
#define MOTOR_E        4
#define MOTOR_F        5

#define MOTOR_BIT_A    0x01
#define MOTOR_BIT_B    0x02
#define MOTOR_BIT_C    0x04
#define MOTOR_BIT_D    0x08
#define MOTOR_BIT_E    0x10
#define MOTOR_BIT_F    0x20
#define MOTOR_BIT_G    0x40
#define MOTOR_BIT_H    0x80

#define MOTOR_ALL      0xFF

#define NUM_COORDS     3

#define COORD_X        0
#define COORD_Y        1
#define COORD_Z        2

typedef enum {MOTOR_IDLE, MOTOR_TRAPEZOIDAL, MOTOR_VELOCITY, MOTOR_OPEN_LOOP} motor_mode_t;

#ifndef MOTOR_STATUS_C

/**
 * 16-bit count registers to count motor steps. Being 16-bit wide, they can
 * store a value between -32768 an 32767.
 */
extern int motor_steps[NUM_MOTORS];

/**
 * Flag that indicates whether a motor is stalled.
 */
extern bool_t motor_stalled[NUM_MOTORS];

extern int motor_desired_pos[NUM_MOTORS];
extern motor_mode_t motor_mode[NUM_MOTORS];
extern char motor_pwm_level[NUM_MOTORS];
extern char motor_direction[NUM_MOTORS];
extern char motor_desired_velocity[NUM_MOTORS];

extern float cartesian_desired_pos[NUM_COORDS];

#endif /* MOTOR_STATUS_C */

#endif /* MOTOR_STATUS_H */
