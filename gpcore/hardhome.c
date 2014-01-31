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
}

inline void hardhome(void)
{
	// FIXME: for each motor, only do hard home if a motor is actually connected.
	// Otherwise, the program would block, waiting for a limit switch to be triggered.
	hardhome_motor_a();
	/*
	hardhome_motor_b();
	hardhome_motor_c();
	hardhome_motor_d();
	hardhome_motor_e();
	hardhome_motor_f();
	*/
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
		// Set PWM level of motor A to 100% duty cycle
		mcuicom_send("PA,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_MA);
		
		// Stop motor
		mcuicom_send("SA" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KA" CMDEND);
	
	// Re-enable PID control on motor A to be able to increment position by a given amount of motor steps
	mcuicom_send("EA" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
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
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "IA,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_A_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor A to the mid-point of points A and B.
	snprintf(buf, size, "GA,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KA" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GA,0" CMDEND);
}

void hardhome_motor_b(void)
{
	int size = 64;
	char buf[size];
	
	// Disable PID control on motor B, to be able to change the PWM duty cycle manually
	mcuicom_send("DB" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_MB)
	{
		// Set PWM level of motor B to 100% duty cycle
		mcuicom_send("PB,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_MB);
		
		// Stop motor
		mcuicom_send("SB" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KB" CMDEND);
	
	// Re-enable PID control on motor B to be able to increment position by a given amount of motor steps
	mcuicom_send("EB" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
	snprintf(buf, size, "IB,%d" CMDEND, STEP_INC);
	while (LMT_MB)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_B_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "IB,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_MB)
		mcuicom_send(buf);
	while (LMT_MB)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_B_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "IB,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_B_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor B to the mid-point of points A and B.
	snprintf(buf, size, "GB,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KB" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GB,0" CMDEND);
}

void hardhome_motor_c(void)
{
	int size = 64;
	char buf[size];
	
	// Disable PID control on motor C, to be able to change the PWM duty cycle manually
	mcuicom_send("DC" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_MC)
	{
		// Set PWM level of motor C to 100% duty cycle
		mcuicom_send("PC,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_MC);
		
		// Stop motor
		mcuicom_send("SC" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KC" CMDEND);
	
	// Re-enable PID control on motor C to be able to increment position by a given amount of motor steps
	mcuicom_send("EC" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
	snprintf(buf, size, "IC,%d" CMDEND, STEP_INC);
	while (LMT_MC)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_C_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "IC,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_MC)
		mcuicom_send(buf);
	while (LMT_MC)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_C_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "IC,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_C_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor C to the mid-point of points A and B.
	snprintf(buf, size, "GC,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KC" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GC,0" CMDEND);
}

void hardhome_motor_d(void)
{
	int size = 64;
	char buf[size];
	
	// Disable PID control on motor D, to be able to change the PWM duty cycle manually
	mcuicom_send("DD" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_MD)
	{
		// Set PWM level of motor D to 100% duty cycle
		mcuicom_send("PD,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_MD);
		
		// Stop motor
		mcuicom_send("SD" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KD" CMDEND);
	
	// Re-enable PID control on motor D to be able to increment position by a given amount of motor steps
	mcuicom_send("ED" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
	snprintf(buf, size, "ID,%d" CMDEND, STEP_INC);
	while (LMT_MD)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_D_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "ID,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_MD)
		mcuicom_send(buf);
	while (LMT_MD)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_D_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "ID,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_D_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor D to the mid-point of points A and B.
	snprintf(buf, size, "GD,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KD" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GD,0" CMDEND);
}

void hardhome_motor_e(void)
{
	int size = 64;
	char buf[size];
	
	// Disable PID control on motor E, to be able to change the PWM duty cycle manually
	mcuicom_send("DE" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_ME)
	{
		// Set PWM level of motor E to 100% duty cycle
		mcuicom_send("PE,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_ME);
		
		// Stop motor
		mcuicom_send("SE" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KE" CMDEND);
	
	// Re-enable PID control on motor E to be able to increment position by a given amount of motor steps
	mcuicom_send("EE" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
	snprintf(buf, size, "IE,%d" CMDEND, STEP_INC);
	while (LMT_ME)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_E_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "IE,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_ME)
		mcuicom_send(buf);
	while (LMT_ME)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_E_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "IE,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_E_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor E to the mid-point of points A and B.
	snprintf(buf, size, "GE,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KE" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GE,0" CMDEND);
}

void hardhome_motor_f(void)
{
	int size = 64;
	char buf[size];
	
	// Disable PID control on motor F, to be able to change the PWM duty cycle manually
	mcuicom_send("DF" CMDEND);
	
	// Stop all motors
	mcuicom_send("SS" CMDEND);
	
	// If the limit switch is not active, search it.
	// To do this, move motor at high speed until the
	// Change Notification interrupt is triggered.
	if (!LMT_MF)
	{
		// Set PWM level of motor F to 100% duty cycle
		mcuicom_send("PF,100" CMDEND);
		
		// Move motor at the specified velocity
		mcuicom_send("MP" CMDEND);
		
		// Wait until the switch is found
		while (!LMT_MF);
		
		// Stop motor
		mcuicom_send("SF" CMDEND);
	}
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KF" CMDEND);
	
	// Re-enable PID control on motor F to be able to increment position by a given amount of motor steps
	mcuicom_send("EF" CMDEND);
	
	// Tune position of the limit switch more finely. To do this, keep moving in the same direction by a few steps
	// at a time until the switch goes off. The move backwards in the same way until the switch goes on and off again.
	// At this point, we have reach both ends of the limit switch and can now compute the mid-point.
	#define STEP_INC 15                             // Increment 15 motor steps at a time
	snprintf(buf, size, "IF,%d" CMDEND, STEP_INC);
	while (LMT_MF)
		mcuicom_send(buf);
	int pointA = get_motor_pos(MOTOR_F_CHAR);       // The one end of the limit switch has been reached. Save position.
	snprintf(buf, size, "IF,%d" CMDEND, -STEP_INC); // Minus sign inverts direction
	while (!LMT_MF)
		mcuicom_send(buf);
	while (LMT_MF)
		mcuicom_send(buf);
	int pointB = get_motor_pos(MOTOR_F_CHAR);       // The other end of the limit switch has been reached. Save position.
	int mid_point = (pointA + pointB) / 2;
	// Move motor to the mid-point of points A and B
	snprintf(buf, size, "IF,%d" CMDEND, STEP_INC);
	#define HH_TOL 10
	while (abs(get_motor_pos(MOTOR_F_CHAR) - mid_point) > HH_TOL)
		mcuicom_send(buf);
	#undef HH_TOL
	#undef STEP_INC
	
	// Set destination position of motor F to the mid-point of points A and B.
	snprintf(buf, size, "GF,%d" CMDEND, mid_point);
	mcuicom_send(buf);
	
	// Clear position register to make PID take the current position as its reference (zero) position
	mcuicom_send("KF" CMDEND);
	
	// Move motor to the hard home position.
	mcuicom_send("GF,0" CMDEND);
}
