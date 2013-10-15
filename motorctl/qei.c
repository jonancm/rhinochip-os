#include "qei.h"

void qei_setup(void)
{
	// Clear output ports
	
	LATB = 0;            // RB0..RB8
	LATCbits.LATC15 = 0; // RC15
	LATD &= ~0b110;      // RD2, RD3
	
	// Set up pins to be used as input ports
	
	TRISB = 1;             // RB0..RB8
	TRISCbits.TRISC15 = 1; // RC15
	TRISD |= 0b110;        // RD2, RD3
	
	// Set up Timer 2 to implement a custom multi-channel QEI
	
	IFS0bits.T2IF = 0; // Clear the timer 2 interrupt flag
	IEC0bits.T2IE = 1; // Enable timer 2 interrupts
	T2CONbits.TCKPS = 0b10; // Set a 1:64 prescale value
	T2CONbits.TON = 1; // Start the timer
}

void __attribute__((__interrupt__)) _T2Interrupt(void)
{
	// Update step count register for motor A
	curr_encoder_state[MOTOR_A] = QEA_MA << 1 | QEB_MA;
	switch (prev_encoder_state[MOTOR_A])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_A])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_A])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_A])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_A])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					break;
			}
			break;
	}
	
	// Update step count register for motor B
	
	// Update step count register for motor C
	
	// Update step count register for motor D
	
	// Update step count register for motor E
	
	// Update step count register for motor F
	
	// Clear interrupt flag
	IFS0bits.T2IF = 0;
}
