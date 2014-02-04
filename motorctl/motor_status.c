#ifndef MOTOR_STATUS_C
#define MOTOR_STATUS_C
#endif

#include "motor_status.h"

/**
 * Conversion factors for joint variables: encoder steps per degree
 */
#define STEPDEG_B      12.800
#define STEPDEG_CDE    35.274
#define STEPDEG_F      17.637

/**
 * Maximum rotational velocity of each motor (encoder steps per milisecond)
 * (6000 rpm) x (360 deg) x (conversion factor) / (60 sec)
 */
#define MAXVEL_B       (0.1 * 360 * STEPDEG_B)
#define MAXVEL_CDE     (0.1 * 360 * STEPDEG_CDE)
#define MAXVEL_F       (0.1 * 360 * STEPDEG_F)

int             motor_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
bool_t          motor_stalled[NUM_MOTORS] = {false, false, false, false, false, false};
int             motor_commanded_pos[NUM_MOTORS];
int             motor_desired_pos[NUM_MOTORS];
motor_mode_t    motor_mode[NUM_MOTORS];
char            motor_pwm_level[NUM_MOTORS];
char            motor_direction[NUM_MOTORS];
char            motor_desired_velocity[NUM_MOTORS] = {100, 100, 100, 100, 100, 100};
float           motor_max_velocity[NUM_MOTORS] = {0, MAXVEL_B, MAXVEL_CDE, MAXVEL_CDE, MAXVEL_CDE, MAXVEL_F};

float           cartesian_desired_pos[NUM_COORDS] = {0, 0, 0};
