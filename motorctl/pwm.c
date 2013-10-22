#include "pwm.h"


/**
 * PWM period.
 */
unsigned int pwmperiod = 0;

/**
 * PWM count register.
 */
struct {
	unsigned int channel4;
	unsigned int channel5;
	unsigned int channel6;
} pwmcount;

/**
 * PWM duty cycle register.
 */
struct {
	unsigned int channel4;
	unsigned int channel5;
	unsigned int channel6;
} pwmduty;

void pwm_setup(void)
{
	/**********************************************
	 * Set up digital I/O pins for digital output *
	 **********************************************/
	
	TRISB = 0;            // All RB0..RB8 are outputs (9 outputs)
	TRISC = 0;            // All RC13, RC14 are outputs (2 outputs)
	TRISEbits.TRISE8 = 0; // RE8 is output (1 output)
	
	/****************************
	 * Initialize PWM registers *
	 ****************************/
	
	*((char*)&pwmenable) = 0;
	
	pwmcount.channel4 = 0;
	pwmcount.channel5 = 0;
	pwmcount.channel6 = 0;
	
	pwmduty.channel4 = 0;
	pwmduty.channel5 = 0;
	pwmduty.channel6 = 0;
	
	/**********************************************************
	 * Set up Timer 1 to implement a custom multi-channel QEI *
	 **********************************************************/
	
	// Clear the timer 1 interrupt flag
	
	IFS0bits.T1IF = 0;
	
	// Enable timer 1 interrupts
	
	IEC0bits.T1IE = 1;
	
	// Set timer 1 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
	
	T1CONbits.TCKPS = 0;
	
	// Set timer 1 interrupt priority to 1 (default: 2)
	
	IPC0bits.T1IP = 1;
	
	// PR1 = (Timer period * fcy) / prescaler =
	//     = (PWM period / 100 * fcy) / prescaler =
	//     = (20 ms / 100 * 30 MHz) / 1 = 6000
	
	PR1 = 6000;
	
	// Start the timer
	
	T1CONbits.TON = 1;
}

void pwm_set_pdc4(int duty)
{
	pwmduty.channel4 = duty;
}

void pwm_set_pdc5(int duty)
{
	pwmduty.channel5 = duty;
}

void pwm_set_pdc6(int duty)
{
	pwmduty.channel6 = duty;
}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	// Disable Timer 1 interrupts while executing ISR
	IEC0bits.T1IE = 0;
	
	// Generate PWM signal on PWM channel 4
	if (pwmenable.channel4)
	{
		if (pwmcount.channel4 < pwmduty.channel4)
		{
			PWM4 = 1;
			++pwmcount.channel4;
		}
		else if (pwmcount.channel4 < PWMRESOL)
		{
			PWM4 = 0;
			++pwmcount.channel4;
		}
		else
		{
			PWM4 = 1;
			pwmcount.channel4 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 5
	if (pwmenable.channel5)
	{
		if (pwmcount.channel5 < pwmduty.channel5)
		{
			PWM5 = 1;
			++pwmcount.channel5;
		}
		else if (pwmcount.channel5 < PWMRESOL)
		{
			PWM5 = 0;
			++pwmcount.channel5;
		}
		else
		{
			PWM5 = 1;
			pwmcount.channel5 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 6
	if (pwmenable.channel6)
	{
		if (pwmcount.channel6 < pwmduty.channel6)
		{
			PWM6 = 1;
			++pwmcount.channel6;
		}
		else if (pwmcount.channel6 < PWMRESOL)
		{
			PWM6 = 0;
			++pwmcount.channel6;
		}
		else
		{
			PWM6 = 1;
			pwmcount.channel6 = 0;
		}
	}
	
	// Clear interrupt flag
	IFS0bits.T1IF = 0;
	
	// Re-enable Timer 1 interrupts
	IEC0bits.T1IE = 1;
}
