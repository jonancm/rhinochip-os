#include "motorctl.h"
#include "pwm.h"
#include "qei.h"
#include "motor_status.h"

#include <string.h> // memset

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
	unsigned int enabled            : 1;
	unsigned int phase              : 1;
	unsigned int velocity_saturated : 1;
	unsigned int flatcount;
	unsigned int velocity;
	unsigned int acceleration;
	unsigned int phase1displacement;
	unsigned int midpoint;
	unsigned int max_velocity;
	int          position;
} motorctl_info_t;

motorctl_info_t    motorctl_info[NUM_MOTORS];

#define SYSTEM_VELOCITY    100

void setup_trapezoidal_movement(void)
{
	// Initialize motor control data structure for motor A
	motorctl_info[MOTOR_A].enabled = false;
	motorctl_info[MOTOR_A].phase = 0;
	motorctl_info[MOTOR_A].velocity_saturated = false;
	motorctl_info[MOTOR_A].flatcount = 0;
	motorctl_info[MOTOR_A].velocity = 0;
	motorctl_info[MOTOR_A].acceleration = 50; // Motor acceleration expressed as percentage of maximum motor velocity (default: SYSTEM_ACCELERATION)
	motorctl_info[MOTOR_A].phase1displacement = 0;
	motorctl_info[MOTOR_A].midpoint = (motor_steps[MOTOR_A] + motor_commanded_pos[MOTOR_A]) / 2;
	motorctl_info[MOTOR_A].max_velocity = SYSTEM_VELOCITY * motor_desired_velocity[MOTOR_A]; // Maximum motor velocity
	motorctl_info[MOTOR_A].position = motor_steps[MOTOR_A];
}

inline void generate_trapezoidal_profile_motor_a(void)
{
	if (motorctl_info[MOTOR_A].enabled)
	{
		// Fall phase (motorctl_info[MOTOR_A].phase = 1)
		if (motorctl_info[MOTOR_A].phase)
		{
			// If the motor velocity IS saturated, keep moving at maximum velocity until duration
			// of segment S2 has been matched
			if (motorctl_info[MOTOR_A].velocity_saturated)
				 --motorctl_info[MOTOR_A].flatcount;
			// If the motor velocity is NOT saturated, decrease velocity at a constant rate
			// (given by system acceleration) until zero velocity is reached
			else
				motorctl_info[MOTOR_A].velocity -= motorctl_info[MOTOR_A].acceleration;
			
			// If segment S3 has finished, unset the saturation flag for the velocity to start
			// decreasing (to enter S4)
			if (motorctl_info[MOTOR_A].flatcount <= 0)
				motorctl_info[MOTOR_A].velocity_saturated = false;
			
			// If the total displacement of the fall phase has been reached (i.e. the end-point
			// of the trajectory has been reached), the fall phase (and the movement) has finished
			if (motorctl_info[MOTOR_A].phase1displacement <= 0)
				motorctl_info[MOTOR_A].enabled = false;
			// If the fall phase has not finished yet, decrease the displacement counter
			else
				--motorctl_info[MOTOR_A].phase1displacement;
		}
		// Rise phase (motorctl_info[MOTOR_A].phase = 0)
		else
		{
			// If the motor velocity IS saturated, keep moving at maximum velocity and count
			// duration of segment S2
			if (motorctl_info[MOTOR_A].velocity_saturated)
				 ++motorctl_info[MOTOR_A].flatcount;
			// If the motor velocity is NOT saturated, increase velocity at a constant rate
			// (given by system acceleration) until maximum velocity is reached
			else
				motorctl_info[MOTOR_A].velocity += motorctl_info[MOTOR_A].acceleration;
			
			// If the maximum velocity has been reached, limit velocity to its maximum value
			// and set saturation flag
			if (motorctl_info[MOTOR_A].velocity >= motorctl_info[MOTOR_A].max_velocity)
			{
				motorctl_info[MOTOR_A].velocity = motorctl_info[MOTOR_A].max_velocity;
				motorctl_info[MOTOR_A].velocity_saturated = true;
			}
			
			// If the motor position has reached the mid-point of the trajectory, the rise phase
			// has finished
			if (motor_steps[MOTOR_A] >= motorctl_info[MOTOR_A].midpoint)
				motorctl_info[MOTOR_A].phase = 1;
			// If the rise phase has not finished yet, increment the displacement counter
			else
				++motorctl_info[MOTOR_A].phase1displacement;
		}
		
		// Calculate next position
		motorctl_info[MOTOR_A].position = motorctl_info[MOTOR_A].position + motorctl_info[MOTOR_A].velocity;
	}
}

void generate_trapezoidal_profile(void)
{
	generate_trapezoidal_profile_motor_a();
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
	// TODO: Set PID gains to the values stored in the EEPROM
	//pid_info[MOTOR_A].KP = 46;
	pid_info[MOTOR_A].KP = 1;
	//pid_info[MOTOR_A].KI = 40;
	pid_info[MOTOR_A].KI = 0;
	//pid_info[MOTOR_A].KD = 110;
	pid_info[MOTOR_A].KD = 0;
}

/***************************
 * Motor control functions *
 ***************************/

inline void motorctl_setup(void)
{
	// Set up data structures for PID position control
	
	setup_pid_info();
	
	// Set up Timer 2 to implement a custom multi-channel QEI
	
	IFS0bits.T3IF = 0; // Clear the timer 3 interrupt flag
	IEC0bits.T3IE = 1; // Enable timer 3 interrupts
	PR3 = PR3VAL;      // Set the timer period
	T3CONbits.TON = 1; // Start the timer
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

void motorctl_move(void)
{
	// Disable PID on all motors, so that no position correction is performed while executing a trapezoidal move
	motorctl_disable_pid(MOTOR_ALL);
	
	// Declare flag to check if all the motors have finished moving
	bool_t move_not_finished = false;
	// Set up the data structure for the trapezoidal velocity profile generation
	setup_trapezoidal_movement();
	// Enable trapezoidal velocity generation for each motor. This makes the motors start moving.
	motorctl_info[MOTOR_A].enabled = true;
	// Perform the trapezoidal move. The microcontrollers blocks until the movement has finished.
	do {
		// Calculate next motor position in order to achieve a trapezoidal velocity profile
		generate_trapezoidal_profile();
		
		// Re-calculate PWM duty cycle using the PID controller
		// TODO: this code is duplicated (has been copied from motorctl), although call to pid_loop has different parameters;
		//       create a separate function to elliminate duplication
		int duty = pid_loop(&pid_info[MOTOR_A], motor_steps[MOTOR_A], motorctl_info[MOTOR_A].position);
		// Translate the PWM duty cycle into a PWM level and a PWM direction
		// and update the corresponding registers
		unsigned char direction;
		duty = abs_sign(duty, &direction);
		direction = 1 - direction; // Uncomment only if 'direction' needs to be inverted
		// Perform the movement
		pwm_set_duty1(duty);
		DIR1 = direction;
		
		// Check if all motors have finished moving and set flag accordingly
		move_not_finished = motorctl_info[MOTOR_A].enabled
		                  | motorctl_info[MOTOR_B].enabled
		                  | motorctl_info[MOTOR_C].enabled
		                  | motorctl_info[MOTOR_D].enabled
		                  | motorctl_info[MOTOR_E].enabled
		                  | motorctl_info[MOTOR_F].enabled;
	} while (move_not_finished);
	
	// Move the contents of 'motor_commanded_pos' to 'motor_desired_pos'
	motor_desired_pos[MOTOR_A] = motor_commanded_pos[MOTOR_A];
	
	// Enable PID on all motors again, for position correction to be performed automatically on a timely basis
	motorctl_enable_pid(MOTOR_ALL);
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
