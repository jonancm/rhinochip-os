#include "motorctl.h"
#include "pwm.h"
#include "qei.h"
#include "motor_status.h"

#include <string.h> // memset

/********************
 * PID control loop *
 ********************/

#define PWM_MIN_DUTY    0
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
	int      pid_output;
	
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
	if (pid_output < PWM_MIN_DUTY)
		pid_output = PWM_MIN_DUTY;
	else if (PWM_MAX_DUTY < pid_output)
		pid_output = PWM_MAX_DUTY;
	
	pid_info->prev_error = pid_info->curr_error;
	
	return pid_output;
}

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
int abs_sign(int num, unsigned char *negative)
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

void setup_pid_info(void)
{
	// Clear the PID information structure for each motor (set memory to zero)
	int i;
	for (i = 0; i < NUM_MOTORS; ++i)
		memset(pid_info, 0, sizeof(pid_info_t));
	// TODO: Enable/disable PID control according to motor mode
	motorctl_enable_pid(MOTOR_ALL);
	// TODO: Set PID gains to the values stored in the EEPROM
	pid_info[MOTOR_A].KP = 25;
	pid_info[MOTOR_A].KI = 5;
	pid_info[MOTOR_A].KD = 10;
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
	
	// Set up data structures for trapezoidal velocity profile generation
	
	// TODO: implement
	// setup_trapezoidal_movement();
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
		duty = abs_sign(duty, &direction);
		//direction = 1 - direction; // Uncomment only if 'direction' needs to be inverted
		/*
		motor_pwm_level[MOTOR_A] = duty;
		motor_direction[MOTOR_A] = direction;
		// These registers are for open-loop mode only
		*/
		pwm_set_duty1(duty);
		DIR1 = direction;
	}
}

void __attribute__((interrupt, auto_psv)) _T3Interrupt(void)
{
	// Disable Timer 3 interrupts
	IEC0bits.T3IE = 0;
	
	// Run PID loop
	motorctl();
	
	// Clear Timer 3 interrupt flag
	IFS0bits.T3IF = 0;
	// Re-enable Timer 3 interrupt flag
	IEC0bits.T3IE = 1;
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
