#include "qei.h"

/**
 * Buffer to store the previous encoder state for each of the six motors, i.e.,
 * the values of QEA and QEB represented as a 2-bit number (b00, b01, b10, b11).
 */
char prev_encoder_state[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

/**
 * Buffer to store the current encoder state for each of the six motors, i.e.,
 * the values of QEA and QEB represented as a 2-bit number (b00, b01, b10, b11).
 */
char curr_encoder_state[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

inline void qei_setup(void)
{
	// Set up digital I/O pins for digital input
	
	ADPCFG |= 0x003F; // Disable analog input on pins RB0..RB5 and enable digital input instead
	
	// Initialize previous encoder state for all motors
	
	prev_encoder_state[MOTOR_A] = (QEA_MA << 1) | QEB_MA;
	prev_encoder_state[MOTOR_B] = (QEA_MB << 1) | QEB_MB;
	prev_encoder_state[MOTOR_C] = (QEA_MC << 1) | QEB_MC;
	prev_encoder_state[MOTOR_D] = (QEA_MD << 1) | QEB_MD;
	prev_encoder_state[MOTOR_E] = (QEA_ME << 1) | QEB_ME;
	prev_encoder_state[MOTOR_F] = (QEA_MF << 1) | QEB_MF;
	
	curr_encoder_state[MOTOR_A] = (QEA_MA << 1) | QEB_MA;
	curr_encoder_state[MOTOR_B] = (QEA_MB << 1) | QEB_MB;
	curr_encoder_state[MOTOR_C] = (QEA_MC << 1) | QEB_MC;
	curr_encoder_state[MOTOR_D] = (QEA_MD << 1) | QEB_MD;
	curr_encoder_state[MOTOR_E] = (QEA_ME << 1) | QEB_ME;
	curr_encoder_state[MOTOR_F] = (QEA_MF << 1) | QEB_MF;
	
	// Set up Timer 2 to implement a custom multi-channel QEI
	
	IFS0bits.T2IF = 0; // Clear the timer 2 interrupt flag
	IEC0bits.T2IE = 1; // Enable timer 2 interrupts
	PR2 = PR2VAL;      // Set the timer period
	T2CONbits.TON = 1; // Start the timer
}

void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
	// Disable timer 2 interrupts
	IEC0bits.T2IE = 0;
	
	// Update step count register for motor A
	prev_encoder_state[MOTOR_A] = curr_encoder_state[MOTOR_A];
	curr_encoder_state[MOTOR_A] = (QEA_MA << 1) | QEB_MA;
	switch (prev_encoder_state[MOTOR_A])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_A])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_A] >= MOTOR_A_MAX_RANGE)
						motor_stalled[MOTOR_A] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_A] <= MOTOR_A_MIN_RANGE)
						motor_stalled[MOTOR_A] = true;
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
					if (--motor_steps[MOTOR_A] <= MOTOR_A_MIN_RANGE)
						motor_stalled[MOTOR_A] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_A] >= MOTOR_A_MAX_RANGE)
						motor_stalled[MOTOR_A] = true;
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
					if (++motor_steps[MOTOR_A] >= MOTOR_A_MAX_RANGE)
						motor_stalled[MOTOR_A] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_A] <= MOTOR_A_MIN_RANGE)
						motor_stalled[MOTOR_A] = true;
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
					if (--motor_steps[MOTOR_A] <= MOTOR_A_MIN_RANGE)
						motor_stalled[MOTOR_A] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_A] >= MOTOR_A_MAX_RANGE)
						motor_stalled[MOTOR_A] = true;
					break;
			}
			break;
	}
	
	// Update step count register for motor B
	prev_encoder_state[MOTOR_B] = curr_encoder_state[MOTOR_B];
	curr_encoder_state[MOTOR_B] = (QEA_MB << 1) | QEB_MB;
	switch (prev_encoder_state[MOTOR_B])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_B])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_B] >= MOTOR_B_MAX_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_B] <= MOTOR_B_MIN_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_B])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_B] <= MOTOR_B_MIN_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_B] >= MOTOR_B_MAX_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_B])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					if (++motor_steps[MOTOR_B] >= MOTOR_B_MAX_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_B] <= MOTOR_B_MIN_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_B])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_B] <= MOTOR_B_MIN_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_B] >= MOTOR_B_MAX_RANGE)
						motor_stalled[MOTOR_B] = true;
					break;
			}
			break;
	}
	
	// Update step count register for motor C
	prev_encoder_state[MOTOR_C] = curr_encoder_state[MOTOR_C];
	curr_encoder_state[MOTOR_C] = (QEA_MC << 1) | QEB_MC;
	switch (prev_encoder_state[MOTOR_C])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_C])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_C] >= MOTOR_C_MAX_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_C] <= MOTOR_C_MIN_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_C])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_C] <= MOTOR_C_MIN_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_C] >= MOTOR_C_MAX_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_C])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					if (++motor_steps[MOTOR_C] >= MOTOR_C_MAX_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_C] <= MOTOR_C_MIN_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_C])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_C] <= MOTOR_C_MIN_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_C] >= MOTOR_C_MAX_RANGE)
						motor_stalled[MOTOR_C] = true;
					break;
			}
			break;
	}
	
	// Update step count register for motor D
	prev_encoder_state[MOTOR_D] = curr_encoder_state[MOTOR_D];
	curr_encoder_state[MOTOR_D] = (QEA_MD << 1) | QEB_MD;
	switch (prev_encoder_state[MOTOR_D])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_D])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_D] >= MOTOR_D_MAX_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_D] <= MOTOR_D_MIN_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_D])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_D] <= MOTOR_D_MIN_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_D] >= MOTOR_D_MAX_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_D])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					if (++motor_steps[MOTOR_D] >= MOTOR_D_MAX_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_D] <= MOTOR_D_MIN_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_D])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_D] <= MOTOR_D_MIN_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_D] >= MOTOR_D_MAX_RANGE)
						motor_stalled[MOTOR_D] = true;
					break;
			}
			break;
	}
	
	// Update step count register for motor E
	prev_encoder_state[MOTOR_E] = curr_encoder_state[MOTOR_E];
	curr_encoder_state[MOTOR_E] = (QEA_ME << 1) | QEB_ME;
	switch (prev_encoder_state[MOTOR_E])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_E])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_E] >= MOTOR_E_MAX_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_E] <= MOTOR_E_MIN_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_E])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_E] <= MOTOR_E_MIN_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_E] >= MOTOR_E_MAX_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_E])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					if (++motor_steps[MOTOR_E] >= MOTOR_E_MAX_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_E] <= MOTOR_E_MIN_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_E])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_E] <= MOTOR_E_MIN_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_E] >= MOTOR_E_MAX_RANGE)
						motor_stalled[MOTOR_E] = true;
					break;
			}
			break;
	}
	
	// Update step count register for motor F
	prev_encoder_state[MOTOR_F] = curr_encoder_state[MOTOR_F];
	curr_encoder_state[MOTOR_F] = (QEA_MF << 1) | QEB_MF;
	switch (prev_encoder_state[MOTOR_F])
	{
		// Previous QEA = 0, QEB = 0
		case 0b00:
			switch (curr_encoder_state[MOTOR_F])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// increasing/clockwise
					if (++motor_steps[MOTOR_F] >= MOTOR_F_MAX_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_F] <= MOTOR_F_MIN_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
			}
			break;
		// Previous QEA = 0, QEB = 1
		case 0b01:
			switch (curr_encoder_state[MOTOR_F])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_F] <= MOTOR_F_MIN_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// increasing/clockwise
					if (++motor_steps[MOTOR_F] >= MOTOR_F_MAX_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 0
		case 0b10:
			switch (curr_encoder_state[MOTOR_F])
			{
				// Current QEA = 0, QEB = 0
				case 0b00:
					// increasing/clockwise
					if (++motor_steps[MOTOR_F] >= MOTOR_F_MAX_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
				// Current QEA = 1, QEB = 1
				case 0b11:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_F] <= MOTOR_F_MIN_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
			}
			break;
		// Previous QEA = 1, QEB = 1
		case 0b11:
			switch (curr_encoder_state[MOTOR_F])
			{
				// Current QEA = 0, QEB = 1
				case 0b01:
					// decreasing/anti-clockwise
					if (--motor_steps[MOTOR_F] <= MOTOR_F_MIN_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
				// Current QEA = 1, QEB = 0
				case 0b10:
					// increasing/clockwise
					if (++motor_steps[MOTOR_F] >= MOTOR_F_MAX_RANGE)
						motor_stalled[MOTOR_F] = true;
					break;
			}
			break;
	}
	
	// Clear interrupt flag
	IFS0bits.T2IF = 0;
	
	// Re-enable timer 2 interrupts
	IEC0bits.T2IE = 1;
}
