#include "pwm.h"

/**
 * PWM period.
 */
unsigned int pwmperiod = 0;

/**
 * PWM count register.
 */
struct {
	#ifndef HARDWARE_PWM
	unsigned int channel1;
	unsigned int channel2;
	unsigned int channel3;
	#endif
	unsigned int channel4;
	unsigned int channel5;
	unsigned int channel6;
} pwmcount;

/**
 * PWM duty cycle register.
 */
struct {
	#ifndef HARDWARE_PWM
	unsigned int channel1;
	unsigned int channel2;
	unsigned int channel3;
	#endif
	unsigned int channel4;
	unsigned int channel5;
	unsigned int channel6;
} pwmduty;

inline void pwm_setup(void)
{
	/******************************
	 * Set up hardware PWM module *
	 ******************************/
	
	#ifdef HARDWARE_PWM

	// Set PWM output pins for independent output mode
	
	PWMCON1bits.PMOD1 = 1;
	PWMCON1bits.PMOD2 = 1;
	PWMCON1bits.PMOD3 = 1;
	
	// Enable RE0, RE2 and RE4 for hardware PWM
	
	PWMCON1bits.PEN1L = 1;
	PWMCON1bits.PEN2L = 1;
	PWMCON1bits.PEN3L = 1;
	
	// Set RE1, RE3 and RE5 to be used as general purpose I/O pins
	
	PWMCON1bits.PEN1H = 0;
	PWMCON1bits.PEN2H = 0;
	PWMCON1bits.PEN3H = 0;
	
	// Set prescale of 1:64
	
	PTCONbits.PTCKPS = 3;
	
	// Set PWM period
	
	PTPER = PWMPER;
	
	// Enable the hardware PWM module
	
	PTCONbits.PTEN = 1;

	#endif
	
	/**********************************************
	 * Set up digital I/O pins for digital output *
	 **********************************************/
	
	TRISB &= 0xFE3F; // RB6, RB7, RB8 are outputs (3 for DIR)
	TRISC  = 0;      // RC13, RC14 are outputs (2 for DIR)
	#ifdef HARDWARE_PWM
	TRISE &= 0xFED5; // RE1, RE3, RE5, RE8 are outputs (3 for software PWM, 1 for DIR)
	#else
	TRISE &= 0xFEC0; // RE0-RE5, RE8 are outputs (6 for software PWM, 1 for DIR)
	#endif
	
	/*************************************
	 * Initialize software PWM registers *
	 *************************************/
	
	#ifndef HARDWARE_PWM
	pwmcount.channel1 = 0;
	pwmcount.channel2 = 0;
	pwmcount.channel3 = 0;
	#endif
	pwmcount.channel4 = 0;
	pwmcount.channel5 = 0;
	pwmcount.channel6 = 0;
	
	#ifndef HARDWARE_PWM
	pwmduty.channel1 = 0;
	pwmduty.channel2 = 0;
	pwmduty.channel3 = 0;
	#endif
	pwmduty.channel4 = 0;
	pwmduty.channel5 = 0;
	pwmduty.channel6 = 0;
	
	/*******************************************************************
	 * Set up Timer 1 to implement a multi-channel software PWM module *
	 *******************************************************************/
	
	// Clear the Timer 1 interrupt flag
	
	IFS0bits.T1IF = 0;
	
	// Enable Timer 1 interrupts
	
	IEC0bits.T1IE = 1;
	
	// Set Timer 1 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)
	
	T1CONbits.TCKPS = 0;
	
	// Set Timer 1 interrupt priority to 1 (default: 2)
	
	IPC0bits.T1IP = 1;
	
	// Set Timer 1 period
	
	PR1 = PR1VAL;
	
	// Start Timer 1
	
	T1CONbits.TON = 1;
}

inline void pwm_set_duty1(int duty)
{
	#ifdef HARDWARE_PWM
	PDC1 = (duty / 100.0) * 2 * PTPER;
	#else
	pwmduty.channel1 = duty;
	#endif
}

inline void pwm_set_duty2(int duty)
{
	#ifdef HARDWARE_PWM
	PDC2 = (duty / 100.0) * 2 * PTPER;
	#else
	pwmduty.channel2 = duty;
	#endif
}

inline void pwm_set_duty3(int duty)
{
	#ifdef HARDWARE_PWM
	PDC3 = (duty / 100.0) * 2 * PTPER;
	#else
	pwmduty.channel3 = duty;
	#endif
}

inline void pwm_set_duty4(int duty)
{
	pwmduty.channel4 = duty;
}

inline void pwm_set_duty5(int duty)
{
	pwmduty.channel5 = duty;
}

inline void pwm_set_duty6(int duty)
{
	pwmduty.channel6 = duty;
}

void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
	// Disable Timer 1 interrupts while executing ISR
	IEC0bits.T1IE = 0;
	
	#ifndef HARDWARE_PWM

	// Generate PWM signal on PWM channel 1
	if (pwmduty.channel1)
	{
		if (pwmcount.channel1 < pwmduty.channel1)
		{
			PWM1 = PWM_ON;
			++pwmcount.channel1;
		}
		else if (pwmcount.channel1 < PWMRESOL)
		{
			PWM1 = PWM_OFF;
			++pwmcount.channel1;
		}
		else
		{
			PWM1 = PWM_ON;
			pwmcount.channel1 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 2
	if (pwmduty.channel2)
	{
		if (pwmcount.channel2 < pwmduty.channel2)
		{
			PWM2 = PWM_ON;
			++pwmcount.channel2;
		}
		else if (pwmcount.channel2 < PWMRESOL)
		{
			PWM2 = PWM_OFF;
			++pwmcount.channel2;
		}
		else
		{
			PWM2 = PWM_ON;
			pwmcount.channel2 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 3
	if (pwmduty.channel3)
	{
		if (pwmcount.channel3 < pwmduty.channel3)
		{
			PWM3 = PWM_ON;
			++pwmcount.channel3;
		}
		else if (pwmcount.channel3 < PWMRESOL)
		{
			PWM3 = PWM_OFF;
			++pwmcount.channel3;
		}
		else
		{
			PWM3 = PWM_ON;
			pwmcount.channel3 = 0;
		}
	}

	#endif /* not defined HARDWARE_PWM */
	
	// Generate PWM signal on PWM channel 4
	if (pwmduty.channel4)
	{
		if (pwmcount.channel4 < pwmduty.channel4)
		{
			PWM4 = PWM_ON;
			++pwmcount.channel4;
		}
		else if (pwmcount.channel4 < PWMRESOL)
		{
			PWM4 = PWM_OFF;
			++pwmcount.channel4;
		}
		else
		{
			PWM4 = PWM_ON;
			pwmcount.channel4 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 5
	if (pwmduty.channel5)
	{
		if (pwmcount.channel5 < pwmduty.channel5)
		{
			PWM5 = PWM_ON;
			++pwmcount.channel5;
		}
		else if (pwmcount.channel5 < PWMRESOL)
		{
			PWM5 = PWM_OFF;
			++pwmcount.channel5;
		}
		else
		{
			PWM5 = PWM_ON;
			pwmcount.channel5 = 0;
		}
	}
	
	// Generate PWM signal on PWM channel 6
	if (pwmduty.channel6)
	{
		if (pwmcount.channel6 < pwmduty.channel6)
		{
			PWM6 = PWM_ON;
			++pwmcount.channel6;
		}
		else if (pwmcount.channel6 < PWMRESOL)
		{
			PWM6 = PWM_OFF;
			++pwmcount.channel6;
		}
		else
		{
			PWM6 = PWM_ON;
			pwmcount.channel6 = 0;
		}
	}
	
	// Clear interrupt flag
	IFS0bits.T1IF = 0;
	
	// Re-enable Timer 1 interrupts
	IEC0bits.T1IE = 1;
}
