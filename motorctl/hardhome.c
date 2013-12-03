#include "hardhome.h"
#include "pwm.h"
#include "motor_status.h"

// debug
#include "../mcuicom.h"
#include "../macros.h"

typedef enum {
	HH_START,
	HH_FIND_SWITCH, // Finding limit switch
	HH_FIND_B,      // Finding point B (past end of limit switch)
	HH_FIND_A1,     // Finding point A (before limit switch, LMT 1 => 0)
	HH_FIND_A2,     // Finding point A (before limit switch, LMT 0 => 1)
	HH_END
} hhstate_t;

hhstate_t    hardhome_state[NUM_MOTORS] = {HH_START, HH_START, HH_START, HH_START, HH_START, HH_START};
int          point_a[NUM_MOTORS];
int          point_b[NUM_MOTORS];

unsigned char prev_port_state;
unsigned char curr_port_state;

inline void hardhome_setup(void)
{
	/*******************************************
	 * Set up I/O pin for QEI/LMT multiplexing *
	 *******************************************/
	
	// Set RF6 to be an output pin
	
	TRISFbits.TRISF6 = 0;
	
	// Set RF6 to default value
	
	LMT_SEL = SEL_QEA;
	
	/********************************************************
	 * Set up Change Notification for the involved I/O pins *
	 ********************************************************/
	
	// Enable RB0..RB5 to interrupt the CPU on input change (CN2..CN7)
	
	CNEN1 |= LMT_MSK;
	
	/***********************************
	 * Initialize port state variables *
	 ***********************************/
	
	prev_port_state = PORTB & LMT_MSK;
	curr_port_state = PORTB & LMT_MSK;
}

inline void hardhome(void)
{
	// Select LMT lines from QEI-LMT multiplexer
	
	LMT_SEL = SEL_LMT;
	
	/********************************************************
	 * Set up Change Notification for the involved I/O pins *
	 ********************************************************/
	
	// Clear the Change Notification interrupt flag
	
	IFS0bits.CNIF = 0;
	
	// Enable Change Notification interrupt
	
	IEC0bits.CNIE = 1;
	// Enabling the Change Notification interrupt probably causes the interrupt to be
	// triggered. Thus, the ISR must either check if it's the first time it runs (and thus
	// don't do anything), or check if any of the inputs has really changed (to ensure that
	// the interrupt hasn't been triggered by the previous instruction, but rather by an
	// actual change in any of the inputs).
	// [See ISR below]
	
	/********************************
	 * Perform Hard Home on motor A *
	 ********************************/
	
	// Block execution while the hard home is in progress.
	// The hard home process is executed by the Change Notification ISR, which has been
	// triggered in the previous instruction, where the interrupts were enabled.
	while (hardhome_state[MOTOR_A] != HH_END);
	
	// Compute mid-point of points A and B. That will be the assumed position of the limit switch, i.e.
	// the hard home position.
	int mid_point = (point_a[MOTOR_A] + point_b[MOTOR_A]) / 2;
	
	// TODO: Move motor to hard home position
	
	// Reset motor steps count
	motor_steps[MOTOR_A] = 0;
	
	/***********************************
	 * Finish and restore intial state *
	 ***********************************/
	
	// Hard home has finished. Disable Change Notification interrupt
	
	IEC0bits.CNIE = 0;
	
	// Restore selection to QEA lines in QEI-LMT multiplexer
	
	LMT_SEL = SEL_QEA;
	
	// Re-initialize the hard home automaton to the initial state
	
	hardhome_state[MOTOR_A] = HH_START;
}

/**
 * Change Notification ISR
 */
void __attribute__((interrupt, auto_psv)) _CNInterrupt(void)
{
	// Disable Change Notification interrupts
	IEC0bits.CNIE = 0;
	
	// Update current port state to the actual state
	curr_port_state = PORTB & LMT_MSK;
	// Compute if there have been any changes in the state of the limit switches.
	// The way to check for changes is to operate the previous state and the current
	// state with an XOR operation. The result will have a logical 1 in every position
	// where a bit change has occurred, while positions where no bit change has occurred
	// will remain set to logical 0.
	unsigned char pin_changes = curr_port_state ^ prev_port_state;
	// If the current port state shows changes with respect to its previous state,
	// process accordingly.
	switch (pin_changes)
	{
		case 0:
			// There are no changes in any port, thus don't do anything.
			// This should actually not happend, because the interrupt
			// should only be triggered when there is a pin change.
			// However, this check is done for added safety, and because
			// we suspect the interrupt is being triggered even when there
			// is no pin change (probably when enabling the interrupt).
			break;
		case LMT_MA_MSK:
			switch (hardhome_state[MOTOR_A])
			{
				case HH_START:
					// If LMT = 1, the limit switch has already been found, thus, jump directly
					// to 2nd phase of the hard home process: finding the point B at a low speed
					if (LMT_MA)
					{
						hardhome_state[MOTOR_A] = HH_FIND_B;
						pwm_set_duty1(HARDHOME_PWM_LEVEL2);
					}
					// If LMT = 0, the limit switch has not been found yet, thus, proceed with
					// 1st phase of the hard home process: finding the limit switch at high speed
					else
					{
						hardhome_state[MOTOR_A] = HH_FIND_SWITCH;
						pwm_set_duty1(HARDHOME_PWM_LEVEL1);
					}
					break;
				case HH_FIND_SWITCH:
					// If LMT = 0 -> 1, the limit switch has been found. Thus, proceed with the next step:
					// finding point B at a low speed
					if (LMT_MA)
					{
						hardhome_state[MOTOR_A] = HH_FIND_B;
						pwm_set_duty1(HARDHOME_PWM_LEVEL2);
					}
					break;
				case HH_FIND_B:
					// If LMT = 1 -> 0, the limit switch has become inactive, thus, point B has been found.
					// Proceed now with the next step: finding point A (same speed, opposite direction)
					if (!LMT_MA)
					{
						point_b[MOTOR_A] = motor_steps[MOTOR_A];
						DIR1 = ~DIR1;
						hardhome_state[MOTOR_A] = HH_FIND_A1;
					}
					break;
				case HH_FIND_A1:
					// While searching for point A, the switch goes from inactive to active (LMT = 0 -> 1)
					// and then from active to inactive (LMT = 0 -> 1). That's when point A is reached (when
					// the second transition occurrs). The first transition must be ignored. Thus, break.
					break;
				case HH_FIND_A2:
					// If LMT = 1 -> 0, the limit switch has become inactive, thus, point A has been found.
					// We're done. Now just finish the process and return control to the hard home routine.
					if (!LMT_MA)
					{
						point_a[MOTOR_A] = motor_steps[MOTOR_A];
						hardhome_state[MOTOR_A] = HH_END;
					}
					break;
			}
			break;
		case LMT_MB_MSK:
			// Not yet implemented
			// TODO: implement
			break;
		case LMT_MC_MSK:
			// Not yet implemented
			// TODO: implement
			break;
		case LMT_MD_MSK:
			// Not yet implemented
			// TODO: implement
			break;
		case LMT_ME_MSK:
			// Not yet implemented
			// TODO: implement
			break;
		case LMT_MF_MSK:
			// Not yet implemented
			// TODO: implement
			break;
	}
	
	// Re-enable Change Notification interrupts
	IEC0bits.CNIE = 1;
	// Clear the Change Notification interrupt flag
	IFS0bits.CNIF = 0;
}
