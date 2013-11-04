#include <p30fxxxx.h>

// Set processor configuration bits for the dsPIC30F4011
_FOSC(CSW_FSCM_OFF & XT_PLL16); // Fosc = 16x 7.37 MHz, Fcy = 29.50 MHz
_FWDT(WDT_OFF);                 // Turn off the watchdog timer
_FBORPOR(MCLR_EN & PWRT_OFF);   // Enable reset pin and turn off the power-up timers.

#include "../delay.h"
#include "pwm.h"
#include "qei.h"

int main(void)
{
	pwm_setup();
	qei_setup();
	
	pwm_set_duty1(75);
	pwm_set_duty2(75);
	pwm_set_duty3(75);
	pwm_set_duty4(75);
	pwm_set_duty5(75);
	pwm_set_duty6(75);
	
	while (1);
	
	return 0;
}
