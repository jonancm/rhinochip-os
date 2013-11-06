#ifndef MOTOR_STATUS_C
#define MOTOR_STATUS_C
#endif

#include "motor_status.h"

int             motor_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
bool_t          motor_stalled[NUM_MOTORS] = {false, false, false, false, false, false};
int             motor_desired_pos[NUM_MOTORS];
motor_mode_t    motor_mode[NUM_MOTORS];
char            motor_pwm_level[NUM_MOTORS];
char            motor_direction[NUM_MOTORS];
char            motor_desired_velocity[NUM_MOTORS];
