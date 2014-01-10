#include "hardhome.h"

#include "../mcuicom.h"
#include "../hostcmdset.h"

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

void hardhome_motor_a(void)
{
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
	
	// Re-enable PID control on motor A
	mcuicom_send("EA" CMDEND);
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
