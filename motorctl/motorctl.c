#include "motorctl.h"
#include "pwm.h"
#include "qei.h"
#include "motor_status.h"

#include <string.h> // memset
#include <stdlib.h> // abs

#include "../debug.h"

/***********************
 * Auxiliary functions *
 ***********************/

/**
 * Return the absolute value and the sign of a given integer quantity.
 * 
 * The absolute value of the quantity is returned by means of the function's return value,
 * while the sign is returned using an input/output parameter (pointer to a variable) which
 * represents a flag that indicates whether the number is negative (flag = 1) or positive
 * (flag = 0).
 * 
 * @param num         integer quantity
 * @param negative    pointer to the variable where the sign flag should be returned
 *                    (1 = the quantity is negative, 0 = the quantity is positive)
 * 
 * @returns           absolute value of the given integer quantity
 */
int abs_neg(int num, unsigned char *negative)
{
	if (num < 0)
	{
		*negative = 1;
		return -num;
	}
	else
	{
		*negative = 0;
		return num;
	}
}

/**
 * Return the absolute value and the sign of a given integer quantity.
 * 
 * The absolute value of the quantity is returned by means of the function's return value,
 * while the sign is returned using an input/output parameter (pointer to a variable).
 * 
 * @param num    integer quantity
 * @param sig    pointer to the variable where the sign should be returned (-1 or 1)
 * 
 * @returns      absolute value of the given integer quantity
 */
int abs_sign(int num, int *sig)
{
	if (num < 0)
	{
		*sig = -1;
		return -num;
	}
	else
	{
		*sig = 1;
		return num;
	}
}

/*********************************************
 * Velocity profile generation (trapezoidal) *
 *********************************************/

typedef struct {
	int enabled            : 1;
	int phase              : 1;
	int velocity_saturated : 1;
	int flatcount;
	int velocity;
	int acceleration;
	int phase1displacement;
	int halfcount;
	int max_velocity;
	int position;
	int start_pos;
	int motion_dir;
} motorctl_info_t;

motorctl_info_t    motorctl_info[NUM_MOTORS];

#define TOL_MOVING    100 /* Tolerance to detect changes in position when the motor is moving */
#define TOL_STEADY    100 /* Tolerance to detect changes in position when the motor is steady */

void setup_trapezoidal_movement(void)
{
	int absval, sig;

	// Initialize motor control data structure for motor A
	absval = abs_sign(motor_commanded_pos[MOTOR_A] - motor_steps[MOTOR_A], &sig);
	motorctl_info[MOTOR_A].enabled = false;
	motorctl_info[MOTOR_A].phase = 0;
	motorctl_info[MOTOR_A].velocity_saturated = false;
	motorctl_info[MOTOR_A].flatcount = 0;
	motorctl_info[MOTOR_A].velocity = 0;
	motorctl_info[MOTOR_A].phase1displacement = 0;
	motorctl_info[MOTOR_A].halfcount = absval / 2;
	motorctl_info[MOTOR_A].max_velocity = (motor_desired_velocity[MOTOR_A] * system_velocity) / 100.0;
	motorctl_info[MOTOR_A].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_A].max_velocity;
	motorctl_info[MOTOR_A].position = motor_steps[MOTOR_A];
	motorctl_info[MOTOR_A].start_pos = motor_steps[MOTOR_A];
	motorctl_info[MOTOR_A].motion_dir = sig;

	// Initialize motor control data structure for motor B
	absval = abs_sign(motor_commanded_pos[MOTOR_B] - motor_steps[MOTOR_B], &sig);
	motorctl_info[MOTOR_B].enabled = false;
	motorctl_info[MOTOR_B].phase = 0;
	motorctl_info[MOTOR_B].velocity_saturated = false;
	motorctl_info[MOTOR_B].flatcount = 0;
	motorctl_info[MOTOR_B].velocity = 0;
	motorctl_info[MOTOR_B].phase1displacement = 0;
	motorctl_info[MOTOR_B].halfcount = absval / 2;
	motorctl_info[MOTOR_B].max_velocity = (motor_desired_velocity[MOTOR_B] * system_velocity) / 100.0;
	motorctl_info[MOTOR_B].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_B].max_velocity;
	motorctl_info[MOTOR_B].position = motor_steps[MOTOR_B];
	motorctl_info[MOTOR_B].start_pos = motor_steps[MOTOR_B];
	motorctl_info[MOTOR_B].motion_dir = sig;

	// Initialize motor control data structure for motor C
	absval = abs_sign(motor_commanded_pos[MOTOR_C] - motor_steps[MOTOR_C], &sig);
	motorctl_info[MOTOR_C].enabled = false;
	motorctl_info[MOTOR_C].phase = 0;
	motorctl_info[MOTOR_C].velocity_saturated = false;
	motorctl_info[MOTOR_C].flatcount = 0;
	motorctl_info[MOTOR_C].velocity = 0;
	motorctl_info[MOTOR_C].phase1displacement = 0;
	motorctl_info[MOTOR_C].halfcount = absval / 2;
	motorctl_info[MOTOR_C].max_velocity = (motor_desired_velocity[MOTOR_C] * system_velocity) / 100.0;
	motorctl_info[MOTOR_C].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_C].max_velocity;
	motorctl_info[MOTOR_C].position = motor_steps[MOTOR_C];
	motorctl_info[MOTOR_C].start_pos = motor_steps[MOTOR_C];
	motorctl_info[MOTOR_C].motion_dir = sig;

	// Initialize motor control data structure for motor D
	absval = abs_sign(motor_commanded_pos[MOTOR_D] - motor_steps[MOTOR_D], &sig);
	motorctl_info[MOTOR_D].enabled = false;
	motorctl_info[MOTOR_D].phase = 0;
	motorctl_info[MOTOR_D].velocity_saturated = false;
	motorctl_info[MOTOR_D].flatcount = 0;
	motorctl_info[MOTOR_D].velocity = 0;
	motorctl_info[MOTOR_D].phase1displacement = 0;
	motorctl_info[MOTOR_D].halfcount = absval / 2;
	motorctl_info[MOTOR_D].max_velocity = (motor_desired_velocity[MOTOR_D] * system_velocity) / 100.0;
	motorctl_info[MOTOR_D].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_D].max_velocity;
	motorctl_info[MOTOR_D].position = motor_steps[MOTOR_D];
	motorctl_info[MOTOR_D].start_pos = motor_steps[MOTOR_D];
	motorctl_info[MOTOR_D].motion_dir = sig;

	// Initialize motor control data structure for motor E
	absval = abs_sign(motor_commanded_pos[MOTOR_E] - motor_steps[MOTOR_E], &sig);
	motorctl_info[MOTOR_E].enabled = false;
	motorctl_info[MOTOR_E].phase = 0;
	motorctl_info[MOTOR_E].velocity_saturated = false;
	motorctl_info[MOTOR_E].flatcount = 0;
	motorctl_info[MOTOR_E].velocity = 0;
	motorctl_info[MOTOR_E].phase1displacement = 0;
	motorctl_info[MOTOR_E].halfcount = absval / 2;
	motorctl_info[MOTOR_E].max_velocity = (motor_desired_velocity[MOTOR_E] * system_velocity) / 100.0;
	motorctl_info[MOTOR_E].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_E].max_velocity;
	motorctl_info[MOTOR_E].position = motor_steps[MOTOR_E];
	motorctl_info[MOTOR_E].start_pos = motor_steps[MOTOR_E];
	motorctl_info[MOTOR_E].motion_dir = sig;

	// Initialize motor control data structure for motor F
	absval = abs_sign(motor_commanded_pos[MOTOR_F] - motor_steps[MOTOR_F], &sig);
	motorctl_info[MOTOR_F].enabled = false;
	motorctl_info[MOTOR_F].phase = 0;
	motorctl_info[MOTOR_F].velocity_saturated = false;
	motorctl_info[MOTOR_F].flatcount = 0;
	motorctl_info[MOTOR_F].velocity = 0;
	motorctl_info[MOTOR_F].phase1displacement = 0;
	motorctl_info[MOTOR_F].halfcount = absval / 2;
	motorctl_info[MOTOR_F].max_velocity = (motor_desired_velocity[MOTOR_F] * system_velocity) / 100.0;
	motorctl_info[MOTOR_F].acceleration = (system_acceleration / 100.) * motorctl_info[MOTOR_F].max_velocity;
	motorctl_info[MOTOR_F].position = motor_steps[MOTOR_F];
	motorctl_info[MOTOR_F].start_pos = motor_steps[MOTOR_F];
	motorctl_info[MOTOR_F].motion_dir = sig;
}

inline void generate_trapezoidal_profile_motor_a(void)
{
	if (motorctl_info[MOTOR_A].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_A].phase = 1)
		if (motorctl_info[MOTOR_A].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_A].flatcount <= 0)
				motorctl_info[MOTOR_A].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_A].velocity_saturated)
				 --motorctl_info[MOTOR_A].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_A].velocity -= motorctl_info[MOTOR_A].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_A].phase1displacement <= 0 || abs(motor_steps[MOTOR_A] - motor_commanded_pos[MOTOR_A]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_A].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_A] - motor_commanded_pos[MOTOR_A]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_A].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_A].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_A].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_A].velocity >= motorctl_info[MOTOR_A].max_velocity)
			{
				motorctl_info[MOTOR_A].velocity = motorctl_info[MOTOR_A].max_velocity;
				motorctl_info[MOTOR_A].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_A].velocity_saturated)
				 ++motorctl_info[MOTOR_A].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_A].velocity += motorctl_info[MOTOR_A].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_A] - motorctl_info[MOTOR_A].start_pos) >= motorctl_info[MOTOR_A].halfcount)
				motorctl_info[MOTOR_A].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_A].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_A] - motor_commanded_pos[MOTOR_A]) < TOL_MOVING)
				motorctl_info[MOTOR_A].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_A] - motor_commanded_pos[MOTOR_A]) < TOL_MOVING)
				motorctl_info[MOTOR_A].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_A].position = motorctl_info[MOTOR_A].position
		                                + motorctl_info[MOTOR_A].motion_dir * motorctl_info[MOTOR_A].velocity;
	}
}

inline void generate_trapezoidal_profile_motor_b(void)
{
	if (motorctl_info[MOTOR_B].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_B].phase = 1)
		if (motorctl_info[MOTOR_B].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_B].flatcount <= 0)
				motorctl_info[MOTOR_B].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_B].velocity_saturated)
				 --motorctl_info[MOTOR_B].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_B].velocity -= motorctl_info[MOTOR_B].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_B].phase1displacement <= 0 || abs(motor_steps[MOTOR_B] - motor_commanded_pos[MOTOR_B]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_B].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_B] - motor_commanded_pos[MOTOR_B]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_B].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_B].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_B].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_B].velocity >= motorctl_info[MOTOR_B].max_velocity)
			{
				motorctl_info[MOTOR_B].velocity = motorctl_info[MOTOR_B].max_velocity;
				motorctl_info[MOTOR_B].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_B].velocity_saturated)
				 ++motorctl_info[MOTOR_B].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_B].velocity += motorctl_info[MOTOR_B].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_B] - motorctl_info[MOTOR_B].start_pos) >= motorctl_info[MOTOR_B].halfcount)
				motorctl_info[MOTOR_B].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_B].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_B] - motor_commanded_pos[MOTOR_B]) < TOL_MOVING)
				motorctl_info[MOTOR_B].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_B] - motor_commanded_pos[MOTOR_B]) < TOL_MOVING)
				motorctl_info[MOTOR_B].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_B].position = motorctl_info[MOTOR_B].position
		                                + motorctl_info[MOTOR_B].motion_dir * motorctl_info[MOTOR_B].velocity;
	}
}

inline void generate_trapezoidal_profile_motor_c(void)
{
	if (motorctl_info[MOTOR_C].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_C].phase = 1)
		if (motorctl_info[MOTOR_C].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_C].flatcount <= 0)
				motorctl_info[MOTOR_C].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_C].velocity_saturated)
				 --motorctl_info[MOTOR_C].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_C].velocity -= motorctl_info[MOTOR_C].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_C].phase1displacement <= 0 || abs(motor_steps[MOTOR_C] - motor_commanded_pos[MOTOR_C]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_C].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_C] - motor_commanded_pos[MOTOR_C]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_C].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_C].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_C].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_C].velocity >= motorctl_info[MOTOR_C].max_velocity)
			{
				motorctl_info[MOTOR_C].velocity = motorctl_info[MOTOR_C].max_velocity;
				motorctl_info[MOTOR_C].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_C].velocity_saturated)
				 ++motorctl_info[MOTOR_C].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_C].velocity += motorctl_info[MOTOR_C].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_C] - motorctl_info[MOTOR_C].start_pos) >= motorctl_info[MOTOR_C].halfcount)
				motorctl_info[MOTOR_C].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_C].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_C] - motor_commanded_pos[MOTOR_C]) < TOL_MOVING)
				motorctl_info[MOTOR_C].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_C] - motor_commanded_pos[MOTOR_C]) < TOL_MOVING)
				motorctl_info[MOTOR_C].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_C].position = motorctl_info[MOTOR_C].position
		                                + motorctl_info[MOTOR_C].motion_dir * motorctl_info[MOTOR_C].velocity;
	}
}

inline void generate_trapezoidal_profile_motor_d(void)
{
	if (motorctl_info[MOTOR_D].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_D].phase = 1)
		if (motorctl_info[MOTOR_D].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_D].flatcount <= 0)
				motorctl_info[MOTOR_D].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_D].velocity_saturated)
				 --motorctl_info[MOTOR_D].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_D].velocity -= motorctl_info[MOTOR_D].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_D].phase1displacement <= 0 || abs(motor_steps[MOTOR_D] - motor_commanded_pos[MOTOR_D]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_D].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_D] - motor_commanded_pos[MOTOR_D]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_D].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_D].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_D].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_D].velocity >= motorctl_info[MOTOR_D].max_velocity)
			{
				motorctl_info[MOTOR_D].velocity = motorctl_info[MOTOR_D].max_velocity;
				motorctl_info[MOTOR_D].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_D].velocity_saturated)
				 ++motorctl_info[MOTOR_D].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_D].velocity += motorctl_info[MOTOR_D].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_D] - motorctl_info[MOTOR_D].start_pos) >= motorctl_info[MOTOR_D].halfcount)
				motorctl_info[MOTOR_D].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_D].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_D] - motor_commanded_pos[MOTOR_D]) < TOL_MOVING)
				motorctl_info[MOTOR_D].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_D] - motor_commanded_pos[MOTOR_D]) < TOL_MOVING)
				motorctl_info[MOTOR_D].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_D].position = motorctl_info[MOTOR_D].position
		                                + motorctl_info[MOTOR_D].motion_dir * motorctl_info[MOTOR_D].velocity;
	}
}

inline void generate_trapezoidal_profile_motor_e(void)
{
	if (motorctl_info[MOTOR_E].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_E].phase = 1)
		if (motorctl_info[MOTOR_E].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_E].flatcount <= 0)
				motorctl_info[MOTOR_E].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_E].velocity_saturated)
				 --motorctl_info[MOTOR_E].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_E].velocity -= motorctl_info[MOTOR_E].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_E].phase1displacement <= 0 || abs(motor_steps[MOTOR_E] - motor_commanded_pos[MOTOR_E]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_E].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_E] - motor_commanded_pos[MOTOR_E]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_E].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_E].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_E].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_E].velocity >= motorctl_info[MOTOR_E].max_velocity)
			{
				motorctl_info[MOTOR_E].velocity = motorctl_info[MOTOR_E].max_velocity;
				motorctl_info[MOTOR_E].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_E].velocity_saturated)
				 ++motorctl_info[MOTOR_E].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_E].velocity += motorctl_info[MOTOR_E].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_E] - motorctl_info[MOTOR_E].start_pos) >= motorctl_info[MOTOR_E].halfcount)
				motorctl_info[MOTOR_E].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_E].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_E] - motor_commanded_pos[MOTOR_E]) < TOL_MOVING)
				motorctl_info[MOTOR_E].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_E] - motor_commanded_pos[MOTOR_E]) < TOL_MOVING)
				motorctl_info[MOTOR_E].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_E].position = motorctl_info[MOTOR_E].position
		                                + motorctl_info[MOTOR_E].motion_dir * motorctl_info[MOTOR_E].velocity;
	}
}

inline void generate_trapezoidal_profile_motor_f(void)
{
	if (motorctl_info[MOTOR_F].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_F].phase = 1)
		if (motorctl_info[MOTOR_F].phase)
		{
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_F].flatcount <= 0)
				motorctl_info[MOTOR_F].velocity_saturated = false;
			
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_F].velocity_saturated)
				 --motorctl_info[MOTOR_F].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_F].velocity -= motorctl_info[MOTOR_F].acceleration;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_F].phase1displacement <= 0 || abs(motor_steps[MOTOR_F] - motor_commanded_pos[MOTOR_F]) < TOL_MOVING)
			//if (motorctl_info[MOTOR_F].phase1displacement <= 0 || abs(motor_desired_pos[MOTOR_F] - motor_commanded_pos[MOTOR_F]) < TOL_MOVING)
			// This fixes the final position offset
				motorctl_info[MOTOR_F].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_F].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_F].phase = 0)
		else
		{
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_F].velocity >= motorctl_info[MOTOR_F].max_velocity)
			{
				motorctl_info[MOTOR_F].velocity = motorctl_info[MOTOR_F].max_velocity;
				motorctl_info[MOTOR_F].velocity_saturated = true;
			}

			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_F].velocity_saturated)
				 ++motorctl_info[MOTOR_F].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_F].velocity += motorctl_info[MOTOR_F].acceleration;
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (abs(motor_steps[MOTOR_F] - motorctl_info[MOTOR_F].start_pos) >= motorctl_info[MOTOR_F].halfcount)
				motorctl_info[MOTOR_F].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_F].phase1displacement;
			
			// This fixes the final position offset
			/*
			if (abs(motor_steps[MOTOR_F] - motor_commanded_pos[MOTOR_F]) < TOL_MOVING)
				motorctl_info[MOTOR_F].enabled = false;
			*/
			/*
			if (abs(motor_desired_pos[MOTOR_F] - motor_commanded_pos[MOTOR_F]) < TOL_MOVING)
				motorctl_info[MOTOR_F].enabled = false;
			*/
		}
		
		// Calculate next position
		motorctl_info[MOTOR_F].position = motorctl_info[MOTOR_F].position
		                                + motorctl_info[MOTOR_F].motion_dir * motorctl_info[MOTOR_F].velocity;
	}
}

void generate_trapezoidal_profile(void)
{
	generate_trapezoidal_profile_motor_a();
	generate_trapezoidal_profile_motor_b();
	generate_trapezoidal_profile_motor_c();
	generate_trapezoidal_profile_motor_d();
	generate_trapezoidal_profile_motor_e();
	generate_trapezoidal_profile_motor_f();
}

/********************
 * PID control loop *
 ********************/

#define PWM_MIN_DUTY    10
#define PWM_MAX_DUTY    100

typedef struct {
	float prev_error; // error at time k-1 (previous error value)
	float curr_error; // error at time k   (current error value)
	float error_sum;  // sum of all the previous error values (integral term)
	float KP;         // proportional gain (P)
	float KI;         // integral gain     (I)
	float KD;         // differential gain (D)
} pid_info_t;

pid_info_t    pid_info[NUM_MOTORS];
bool_t        pid_enabled[NUM_MOTORS];

int pid_loop(pid_info_t *pid_info, int current_pos, int desired_pos)
{
	float    error_diff;
	int      pid_output, pid_abs, pid_sig;
	
	pid_info->curr_error = desired_pos - current_pos;
	pid_info->error_sum += pid_info->curr_error * T3PERIOD; // Sum error * dt
	
	error_diff = pid_info->curr_error - pid_info->prev_error;
	
	// If the motor has arrived at its desired position (i.e. the error is zero),
	// eliminate the integral and derivative terms by making them zero.
	// Since the error is zero, the proportional term will also be zero.
	// This is intended to eliminate any PWM pulse after the motor has arrived
	// its desired position.
	if (pid_info->curr_error == 0)
	{
		pid_info->error_sum = 0; // make integral term become zero
		error_diff = 0;          // make derivative term become zero
	}
	
	pid_output = pid_info->KP * pid_info->curr_error
	           + pid_info->KI * pid_info->error_sum
	           + pid_info->KD * error_diff * T3FREQ; // d(error) / dt

	/*
	if (PWM_MIN_DUTY <= pid_output && pid_output <= PWM_MAX_DUTY)
		pid_info->error_sum = tmpi;
	else
	*/
	pid_abs = abs_sign(pid_output, &pid_sig);
	if (0 < pid_abs && pid_abs < PWM_MIN_DUTY)
		pid_output = pid_sig * PWM_MIN_DUTY;
	else if (PWM_MAX_DUTY < pid_abs)
		pid_output = pid_sig * PWM_MAX_DUTY;
	
	pid_info->prev_error = pid_info->curr_error;
	
	return pid_output;
}

void setup_pid_info(void)
{
	// Clear the PID information structure for each motor (set memory to zero)
	int i;
	for (i = 0; i < NUM_MOTORS; ++i)
		memset(pid_info, 0, sizeof(pid_info_t));
	// TODO: Enable/disable PID control according to motor mode
	motorctl_enable_pid(MOTOR_ALL);
}

int motorctl_get_proportional_gain(unsigned char motor)
{
	int gain = -1;

	switch (motor)
	{
		case MOTOR_BIT_A: gain = pid_info[MOTOR_A].KP; break;
		case MOTOR_BIT_B: gain = pid_info[MOTOR_B].KP; break;
		case MOTOR_BIT_C: gain = pid_info[MOTOR_C].KP; break;
		case MOTOR_BIT_D: gain = pid_info[MOTOR_D].KP; break;
		case MOTOR_BIT_E: gain = pid_info[MOTOR_E].KP; break;
		case MOTOR_BIT_F: gain = pid_info[MOTOR_F].KP; break;
		default: break;
	}

	return gain;
}

int motorctl_get_integral_gain(unsigned char motor)
{
	int gain = -1;

	switch (motor)
	{
		case MOTOR_BIT_A: gain = pid_info[MOTOR_A].KI; break;
		case MOTOR_BIT_B: gain = pid_info[MOTOR_B].KI; break;
		case MOTOR_BIT_C: gain = pid_info[MOTOR_C].KI; break;
		case MOTOR_BIT_D: gain = pid_info[MOTOR_D].KI; break;
		case MOTOR_BIT_E: gain = pid_info[MOTOR_E].KI; break;
		case MOTOR_BIT_F: gain = pid_info[MOTOR_F].KI; break;
		default: break;
	}

	return gain;
}

int motorctl_get_differential_gain(unsigned char motor)
{
	int gain = -1;

	switch (motor)
	{
		case MOTOR_BIT_A: gain = pid_info[MOTOR_A].KD; break;
		case MOTOR_BIT_B: gain = pid_info[MOTOR_B].KD; break;
		case MOTOR_BIT_C: gain = pid_info[MOTOR_C].KD; break;
		case MOTOR_BIT_D: gain = pid_info[MOTOR_D].KD; break;
		case MOTOR_BIT_E: gain = pid_info[MOTOR_E].KD; break;
		case MOTOR_BIT_F: gain = pid_info[MOTOR_F].KD; break;
		default: break;
	}

	return gain;
}

void motorctl_set_proportional_gain(unsigned char motor, int gain)
{
	switch (motor)
	{
		case MOTOR_BIT_A: pid_info[MOTOR_A].KP = gain; break;
		case MOTOR_BIT_B: pid_info[MOTOR_B].KP = gain; break;
		case MOTOR_BIT_C: pid_info[MOTOR_C].KP = gain; break;
		case MOTOR_BIT_D: pid_info[MOTOR_D].KP = gain; break;
		case MOTOR_BIT_E: pid_info[MOTOR_E].KP = gain; break;
		case MOTOR_BIT_F: pid_info[MOTOR_F].KP = gain; break;
		default: break;
	}
}

void motorctl_set_integral_gain(unsigned char motor, int gain)
{
	switch (motor)
	{
		case MOTOR_BIT_A: pid_info[MOTOR_A].KI = gain; break;
		case MOTOR_BIT_B: pid_info[MOTOR_B].KI = gain; break;
		case MOTOR_BIT_C: pid_info[MOTOR_C].KI = gain; break;
		case MOTOR_BIT_D: pid_info[MOTOR_D].KI = gain; break;
		case MOTOR_BIT_E: pid_info[MOTOR_E].KI = gain; break;
		case MOTOR_BIT_F: pid_info[MOTOR_F].KI = gain; break;
		default: break;
	}
}

void motorctl_set_differential_gain(unsigned char motor, int gain)
{
	switch (motor)
	{
		case MOTOR_BIT_A: pid_info[MOTOR_A].KD = gain; break;
		case MOTOR_BIT_B: pid_info[MOTOR_B].KD = gain; break;
		case MOTOR_BIT_C: pid_info[MOTOR_C].KD = gain; break;
		case MOTOR_BIT_D: pid_info[MOTOR_D].KD = gain; break;
		case MOTOR_BIT_E: pid_info[MOTOR_E].KD = gain; break;
		case MOTOR_BIT_F: pid_info[MOTOR_F].KD = gain; break;
		default: break;
	}
}

/***************************
 * Motor control functions *
 ***************************/

inline void motorctl_setup(void)
{
	// Set up data structures for PID position control
	
	setup_pid_info();
	
	// Set up Timer 3 to implement PID control
	
	IFS0bits.T3IF = 0; // Clear the timer 3 interrupt flag
	IEC0bits.T3IE = 1; // Enable timer 3 interrupts
	PR3 = PR3VAL;      // Set the timer period
	T3CONbits.TON = 1; // Start the timer

	// Set up Timer 4 to implement Trapezoidal Velocity Profile Generation

	IFS1bits.T4IF = 0;   // Clear the Timer 4 interrupt flag
	IEC1bits.T4IE = 1;   // Enable Timer 4 interrupts
	T4CONbits.TCKPS = 3; // Set prescale value (0=1:1, 1=1:8, 2=1:64, 3=1:256)
	PR4 = PR4VAL;        // Set the timer period
}

inline void motorctl(void)
{
	// If the PID control for motor A is enabled, control motor A,
	// correcting its position according to the PID gains.
	if (pid_enabled[MOTOR_A])
	{
		// Re-calculate PWM duty cycle using the PID controller
		int duty = pid_loop(&pid_info[MOTOR_A], motor_steps[MOTOR_A], motor_desired_pos[MOTOR_A]);
		// Translate the PWM duty cycle into a PWM level and a PWM direction
		// and update the corresponding registers
		unsigned char direction;
		duty = abs_neg(duty, &direction);
		direction = 1 - direction; // Uncomment only if 'direction' needs to be inverted
		// Perform the movement
		pwm_set_duty1(duty);
		DIR1 = direction;
	}
}

void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{
	// Disable Timer 3 interrupts
	IEC0bits.T3IE = 0;
	
	dbgmsg_uart1("_T3Interrupt\n");
	
	// Run PID loop
	motorctl();
	
	// Clear Timer 3 interrupt flag
	IFS0bits.T3IF = 0;
	// Re-enable Timer 3 interrupt flag
	IEC0bits.T3IE = 1;
}

bool_t move_finished = true;

void __attribute__((interrupt, auto_psv)) _T4Interrupt(void)
{
	// Disable Timer 4 interrupts

	IEC1bits.T4IE = 0;
	
	// Calculate next motor position in order to achieve a trapezoidal velocity profile

	generate_trapezoidal_profile();

	// Command PID loop to go to the new position

	if (motorctl_info[MOTOR_A].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_A] = motorctl_info[MOTOR_A].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_A] = motor_commanded_pos[MOTOR_A];
	}

	if (motorctl_info[MOTOR_B].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_B] = motorctl_info[MOTOR_B].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_B] = motor_commanded_pos[MOTOR_B];
	}

	if (motorctl_info[MOTOR_C].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_C] = motorctl_info[MOTOR_C].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_C] = motor_commanded_pos[MOTOR_C];
	}

	if (motorctl_info[MOTOR_D].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_D] = motorctl_info[MOTOR_D].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_D] = motor_commanded_pos[MOTOR_D];
	}

	if (motorctl_info[MOTOR_E].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_E] = motorctl_info[MOTOR_E].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_E] = motor_commanded_pos[MOTOR_E];
	}

	if (motorctl_info[MOTOR_F].enabled)
	{
		// TODO: Maybe this (i.e. setting the desired pos only if the motor is
		// enabled) solves the problem of the position offset at the end of the
		// move that was solved above using TOL_MOVING.
		motor_desired_pos[MOTOR_F] = motorctl_info[MOTOR_F].position;
	}
	else
	{
		// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
		motor_desired_pos[MOTOR_F] = motor_commanded_pos[MOTOR_F];
	}
	
	// Check if all motors have finished moving and set flag accordingly

	move_finished = !(motorctl_info[MOTOR_A].enabled
	               || motorctl_info[MOTOR_B].enabled
	               || motorctl_info[MOTOR_C].enabled
	               || motorctl_info[MOTOR_D].enabled
	               || motorctl_info[MOTOR_E].enabled
	               || motorctl_info[MOTOR_F].enabled);
	
	if (move_finished)
	{
		// Stop Timer 4
		T4CONbits.TON = 0;
		
		// Enable PID on all motors again, for position correction to be performed automatically on a timely basis
		// motorctl_enable_pid(MOTOR_ALL);
		// TODO: remove
	}
	
	// Clear Timer 4 interrupt flag

	IFS1bits.T4IF = 0;

	// Re-enable Timer 4 interrupt

	IEC1bits.T4IE = 1;
}

void motorctl_move(void)
{
	// Set up the data structure for the trapezoidal velocity profile generation

	setup_trapezoidal_movement();
	move_finished = false;

	// Enable trapezoidal velocity generation for each motor whose commanded
	// position register has changed with respect to its actual position register.
	// This makes the motors start moving.

	if (abs(motor_commanded_pos[MOTOR_A] - motor_steps[MOTOR_A]) > TOL_STEADY)
		motorctl_info[MOTOR_A].enabled = true;

	if (abs(motor_commanded_pos[MOTOR_B] - motor_steps[MOTOR_B]) > TOL_STEADY)
		motorctl_info[MOTOR_B].enabled = true;

	if (abs(motor_commanded_pos[MOTOR_C] - motor_steps[MOTOR_C]) > TOL_STEADY)
		motorctl_info[MOTOR_C].enabled = true;

	if (abs(motor_commanded_pos[MOTOR_D] - motor_steps[MOTOR_D]) > TOL_STEADY)
		motorctl_info[MOTOR_D].enabled = true;

	if (abs(motor_commanded_pos[MOTOR_E] - motor_steps[MOTOR_E]) > TOL_STEADY)
		motorctl_info[MOTOR_E].enabled = true;

	if (abs(motor_commanded_pos[MOTOR_F] - motor_steps[MOTOR_F]) > TOL_STEADY)
		motorctl_info[MOTOR_F].enabled = true;

	// Activate Timer 4 to perform the trapezoidal move

	IFS1bits.T4IF = 1; // Set Timer 4 interrupt flag, for ISR to be called upon Timer 4 start
	T4CONbits.TON = 1; // Start Timer 4
}

inline bool_t executing_trapezoidal_move(void)
{
	return !move_finished;
}

inline bool_t motor_a_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_A].enabled;
}

inline bool_t motor_b_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_B].enabled;
}

inline bool_t motor_c_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_C].enabled;
}

inline bool_t motor_d_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_D].enabled;
}

inline bool_t motor_e_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_E].enabled;
}

inline bool_t motor_f_executing_trapezoidal_move(void)
{
	return motorctl_info[MOTOR_F].enabled;
}

void motorctl_enable_pid(unsigned char motors)
{
	// Enable PID control for motor A
	if (motors & MOTOR_BIT_A)
		pid_enabled[MOTOR_A] = true;
	
	// Enable PID control for motor B
	if (motors & MOTOR_BIT_B)
		pid_enabled[MOTOR_B] = true;
	
	// Enable PID control for motor C
	if (motors & MOTOR_BIT_C)
		pid_enabled[MOTOR_C] = true;
	
	// Enable PID control for motor D
	if (motors & MOTOR_BIT_D)
		pid_enabled[MOTOR_D] = true;
	
	// Enable PID control for motor E
	if (motors & MOTOR_BIT_E)
		pid_enabled[MOTOR_E] = true;
	
	// Enable PID control for motor F
	if (motors & MOTOR_BIT_F)
		pid_enabled[MOTOR_F] = true;
}

void motorctl_disable_pid(unsigned char motors)
{
	// Disable PID control for motor A
	if (motors & MOTOR_BIT_A)
		pid_enabled[MOTOR_A] = false;
	
	// Disable PID control for motor B
	if (motors & MOTOR_BIT_B)
		pid_enabled[MOTOR_B] = false;
	
	// Disable PID control for motor C
	if (motors & MOTOR_BIT_C)
		pid_enabled[MOTOR_C] = false;
	
	// Disable PID control for motor D
	if (motors & MOTOR_BIT_D)
		pid_enabled[MOTOR_D] = false;
	
	// Disable PID control for motor E
	if (motors & MOTOR_BIT_E)
		pid_enabled[MOTOR_E] = false;
	
	// Disable PID control for motor F
	if (motors & MOTOR_BIT_F)
		pid_enabled[MOTOR_F] = false;
}
