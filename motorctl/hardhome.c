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
	
	mcuicom_send("hardhome\n", STRLEN("hardhome\n"));
	
	/********************************
	 * Perform Hard Home on motor A *
	 ********************************/
	
	// Start moving motor at high speed searching for the limit switch
	
	pwm_set_duty1(HARDHOME_PWM_LEVEL1);
	
	// Wait until the limit switch is activated
	
	mcuicom_send("wait lmt\n", STRLEN("wait lmt\n"));
	while (!LMT_MA);
	mcuicom_send("proceed\n", STRLEN("proceed\n"));
	
	// When the limit switch is activated, reset step counter
	// and go on moving until limit switch is reached again
	
	pwm_set_duty1(0);
	motor_steps[MOTOR_A] = 0;
	pwm_set_duty1(HARDHOME_PWM_LEVEL1);
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
