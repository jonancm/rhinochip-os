#include "pwm.h"

/**
 * PWM channel enable.
 */
struct {
	unsigned channel1 : 1;
	unsigned channel2 : 1;
	unsigned channel3 : 1;
	unsigned channel4 : 1;
	unsigned channel5 : 1;
	unsigned channel6 : 1;
	unsigned          : 2; // padding to complete the byte
} pwmenable;

/**
 * PWM period.
 */
unsigned int pwmperiod = 0;

/**
 * PWM count register.
 */
struct {
	unsigned int channel1;
	unsigned int channel2;
	unsigned int channel3;
	unsigned int channel4;
	unsigned int channel5;
	unsigned int channel6;
} pwmcount;

/**
 * PWM duty cycle register.
 */
struct {
	unsigned int channel1;
	unsigned int channel2;
	unsigned int channel3;
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
	pwmenable.channel1 = 1;
	
	pwmcount.channel1 = 0;
	pwmcount.channel2 = 0;
	pwmcount.channel3 = 0;
	pwmcount.channel4 = 0;
	pwmcount.channel5 = 0;
	pwmcount.channel6 = 0;
	
	pwmduty.channel1 = 0;
	pwmduty.channel2 = 0;
	pwmduty.channel3 = 0;
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

void pwm_set_pdc1(int duty)
{
	pwmduty.channel1 = duty;
}

void __attribute__((__interrupt__)) _T1Interrupt(void)
{
	// Disable Timer 1 interrupts while executing ISR
	IEC0bits.T1IE = 0;
	
	if (pwmenable.channel1)
	{
		if (pwmcount.channel1 < pwmduty.channel1)
		{
			PWM1 = 1;
			++pwmcount.channel1;
		}
		else if (pwmcount.channel1 < PWMRESOL)
		{
			PWM1 = 0;
			++pwmcount.channel1;
		}
		else
		{
			PWM1 = 1;
			pwmcount.channel1 = 0;
		}
	}
	
	// Clear interrupt flag
	IFS0bits.T1IF = 0;
	
	// Re-enable Timer 1 interrupts
	IEC0bits.T1IE = 1;
}
