#include "hardhome.h"
#include "pwm.h"
#include "motor_status.h"

// debug
#include "../mcuicom.h"
#include "../macros.h"

inline void hardhome_setup(void)
{
	// Set RF6 to be an output pin
	
	TRISFbits.TRISF6 = 0;
	
	// Set RF6 to default value
	
	LMT_SEL = SEL_QEA;
}

inline void hardhome(void)
{
	// Select LMT lines from QEI-LMT multiplexer
	LMT_SEL = SEL_LMT;
	
	/********************************
	 * Perform Hard Home on motor A *
	 ********************************/
	
	// Start moving motor at high speed searching for the limit switch
	// until the limit switch is activated
	if (!LMT_MA)
		pwm_set_duty1(HARDHOME_PWM_LEVEL1);
	while (!LMT_MA);
	
	// When the limit switch is activated, reset the step counter
	// and go on moving until the limit switch is reached again
	pwm_set_duty1(0);
	motor_steps[MOTOR_A] = 0;
	pwm_set_duty1(HARDHOME_PWM_LEVEL2);
	while (LMT_MA);
	
	// When the limit switch is reached again, stop the motor and finish
	while (!LMT_MA);
	pwm_set_duty1(0);
	
	/**********
	 * Finish *
	 **********/
	
	// Restore selection to QEA lines in QEI-LMT multiplexer
	
	LMT_SEL = SEL_QEA;
}
