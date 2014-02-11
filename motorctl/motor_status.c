#ifndef MOTOR_STATUS_C
#define MOTOR_STATUS_C
#endif

#include "motor_status.h"

motor_mode_t    motor_mode[NUM_MOTORS];
motor_status_t  motor_status[NUM_MOTORS] = {STOPPED, STOPPED, STOPPED, STOPPED, STOPPED, STOPPED};

int             motor_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
bool_t          motor_stalled[NUM_MOTORS] = {false, false, false, false, false, false};
int             motor_commanded_pos[NUM_MOTORS];
int             motor_desired_pos[NUM_MOTORS];
char            motor_pwm_level[NUM_MOTORS];
char            motor_direction[NUM_MOTORS];
char            motor_desired_velocity[NUM_MOTORS] = {100, 100, 100, 100, 100, 100};

/*
 * Convert maximum velocity of each motor from revolutions per minute to
 * encoder steps per milisecond.
 */
#define STEPS_PER_DEG_B     12.800
#define STEPS_PER_DEG_CDE   35.274
#define STEPS_PER_DEG_F     17.637
float motor_max_velocity[NUM_MOTORS] = {
	-1,
	((6000/1000) * 360 * STEPS_PER_DEG_B)   / 60, // Max vel = 6000 rpm
	((6000/1000) * 360 * STEPS_PER_DEG_CDE) / 60, // Max vel = 6000 rpm
	((6000/1000) * 360 * STEPS_PER_DEG_CDE) / 60, // Max vel = 6000 rpm
	((6000/1000) * 360 * STEPS_PER_DEG_CDE) / 60, // Max vel = 6000 rpm
	((6000/1000) * 360 * STEPS_PER_DEG_F)   / 60  // Max vel = 6000 rpm
};

float           cartesian_desired_pos[NUM_COORDS] = {0, 0, 0};

char            system_velocity = 100;
char            system_acceleration = 25;
