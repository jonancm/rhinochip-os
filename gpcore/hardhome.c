#include "hardhome.h"

#include "mctlcom.h"
#include "../hostcmdset.h"

#include <stdlib.h> // atoi
#include <stdio.h>  // snprintf

#define MOTOR_A_CHAR    'A'
#define MOTOR_B_CHAR    'B'
#define MOTOR_C_CHAR    'C'
#define MOTOR_D_CHAR    'D'
#define MOTOR_E_CHAR    'E'
#define MOTOR_F_CHAR    'F'

inline void lmtswitch_setup(void)
{
	/*********************************************
	 * Set up digital I/O pins for digital input *
	 *********************************************/
	
	// Disable analog input on pins RB0..RB5 and enable digital input instead
	
	ADPCFG |= 0x003F;
	
	/********************************************************
	 * Set up Change Notification for the involved I/O pins *
	 ********************************************************/
	
	// Enable RB0..RB5 to interrupt the CPU on input change (CN2..CN7)
	
	CNEN1 |= 0x00FC;
	
	// Clear the Change Notification interrupt flag
	
	IFS0bits.CNIF = 0;
}

inline void hardhome(void)
{
	hardhome_motor_a();
	hardhome_motor_b();
	hardhome_motor_c();
	hardhome_motor_d();
	hardhome_motor_e();
	hardhome_motor_f();
}

/**
 * Flag to tell the hard home routine that the switch has been reached.
 */
bool_t switch_found = false;

/**
 * Change Notification ISR
 */
void __attribute__((interrupt, auto_psv)) _CNInterrupt(void)
{
	// Enable flag to tell the hard home routine that the switch has been reached
	switch_found = true;
	// Clear the Change Notification interrupt flag
	IFS0bits.CNIF = 0;
}

int get_motor_pos(char motor)
{
	int size = 64, motor_pos = 0;
	char buf[size];
	
	// Send MCUICOM command RA, RB, ..., RF depending on motor letter (param 1)
	buf[0] = 'R';
	buf[1] = motor;
	buf[2] = *CMDEND;
	buf[3] = '\0';
	mcuicom_send(buf);
	// Get response (motor steps) and convert it to integer
	size = mctlcom_get_response(buf, size);
	if (size > 0)
		motor_pos = atoi(buf);

	return motor_pos;
}

void hardhome_motor_a(void)
{
	int size = 64;
	char buf[size];

	// Disable PID control on motor A, to be able to change the PWM duty cycle manually
	mcuicom_send("DA" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_MA)
	{
		// Enable Change Notification interrupts
		IEC0bits.CNIE = 1;
		
		// Set PWM level of motor A to 100% duty cycle
		mcuicom_send("PA,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		//while (!switch_found);
		while (!LMT_MA);
		
		// Stop motor
		mcuicom_send("SA" CMDEND);
		
		// Disable Change Notification interrupts
		IEC0bits.CNIE = 0;
		
		// Reset flag to allow more hard homes to be executed
		switch_found = false;
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KA" CMDEND);
	
	// Re-enable PID control on motor A to be able to increment position by a given amount of motor steps
	mcuicom_send("EA" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 4                              // Increment 4 motor steps at a time
	snprintf(buf, size, "IA,%d" CMDEND, STEP_INC);
	while (LMT_MA)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_A_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "IA,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_MA)
		mcuicom_send(buf);
	while (LMT_MA)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_A_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	#undef STEP_INC
	
	// Move motor to the mid-point of points A and B.
	snprintf(buf, size, "GA,%d" CMDEND, mid_point);
	mcuicom_send(buf);

	// Wait until motor A stops
	while (get_motor_pos(MOTOR_A_CHAR) != mid_point);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KA" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GA,0" CMDEND);
}

void hardhome_motor_b(void)
{
	
}

void hardhome_motor_c(void)
{
	
}

void hardhome_motor_d(void)
{
	
}

void hardhome_motor_e(void)
{
	
}

void hardhome_motor_f(void)
{
	
}
