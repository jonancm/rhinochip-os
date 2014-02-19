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
typedef enum {MOVING_TRAPEZOIDAL, MOVING_PID, STOPPED} motor_status_t;

#ifndef MOTOR_STATUS_C

extern motor_mode_t motor_mode[NUM_MOTORS];
extern motor_status_t motor_status[NUM_MOTORS];

/**
 * 16-bit count registers to count motor steps. Being 16-bit wide, they can
 * store a value between -32768 an 32767.
 */
extern int motor_steps[NUM_MOTORS];

/**
 * Flag that indicates whether a motor is stalled.
 */
extern bool_t motor_stalled[NUM_MOTORS];

/**
 * Position that motor is commanded to go to. This is not yet the desired position.
 * The desired position is where the motor should be when using PID control.
 * The commanded position is a temporary storage for the position indicated by a PD or PR
 * command before this position is taken as a desired position (this happens after the
 * issuance of an MI or MC command).
 */
extern int motor_commanded_pos[NUM_MOTORS];

/**
 * Position where the motor should be. This is used by the PID loop to compensate errors in the position
 * after the trapezoidal move has been completed.
 */
extern int motor_desired_pos[NUM_MOTORS];

extern char motor_pwm_level[NUM_MOTORS];
extern char motor_direction[NUM_MOTORS];
extern char motor_desired_velocity[NUM_MOTORS];
extern float motor_max_velocity[NUM_MOTORS];

extern float cartesian_desired_pos[NUM_COORDS];

extern char system_velocity;
extern char system_acceleration;

#endif /* MOTOR_STATUS_C */

#endif /* MOTOR_STATUS_H */
