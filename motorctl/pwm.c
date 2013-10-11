#include "pwm.h"

void pwm_setup(void)
{
	// Set up PWM pin pairs to be in independent output mode
	
	PWMCON1bits.PMOD1 = 1; // PWM pin pair 1 is in independent output mode
	PWMCON1bits.PMOD2 = 1; // PWM pin pair 1 is in independent output mode
	PWMCON1bits.PMOD3 = 1; // PWM pin pair 1 is in independent output mode
	
	// Enable PWM pins for PWM output
	
	PWMCON1bits.PEN1H = 1; // PWM1H pin is enabled for PWM output
	PWMCON1bits.PEN2H = 1; // PWM2H pin is enabled for PWM output
	PWMCON1bits.PEN3H = 1; // PWM3H pin is enabled for PWM output
	PWMCON1bits.PEN1L = 1; // PWM1L pin is enabled for PWM output
	PWMCON1bits.PEN2L = 1; // PWM2L pin is enabled for PWM output
	PWMCON1bits.PEN3L = 1; // PWM3L pin is enabled for PWM output
	
	// Set prescale of 1:16
	
	PTCONbits.PTCKPS = 2;
	
	// Set up RE output ports (PWM ports) to be used as digital outputs,
	// in order to be able to write to the LATE register
	
	/*
	TRISEbits.TRISE0 = 0; // Set RE0 (PWM1L) as a digital output
	TRISEbits.TRISE1 = 0; // Set RE1 (PWM1H) as a digital output
	TRISEbits.TRISE2 = 0; // Set RE2 (PWM2L) as a digital output
	TRISEbits.TRISE3 = 0; // Set RE3 (PWM2H) as a digital output
	TRISEbits.TRISE4 = 0; // Set RE4 (PWM3L) as a digital output
	TRISEbits.TRISE5 = 0; // Set RE5 (PWM3H) as a digital output
	*/
	
	// Set PWM module in free running mode
	
	// PTCONbits.PTMOD = 0;
	
	// Set PWM period
	
	PTPER = PWMPER;
	
	// Enable the PWM module
	
	PTCONbits.PTEN = 1;
}

void pwm_set_pdc1(int duty)
{
	PDC1 = (duty / 100.0) * 2 * PTPER;
}	
