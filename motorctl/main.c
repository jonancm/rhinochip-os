#include <p30fxxxx.h>

// Configuration settings
_FOSC(CSW_FSCM_OFF & FRC_PLL16); // Fosc=16x7.5MHz, Fcy=30MHz
_FWDT(WDT_OFF);                  // Watchdog timer off
_FBORPOR(MCLR_DIS);              // Disable reset pin

#include "../delay.h"
#include "pwm.h"

int main(void)
{
	
	pwm_setup();
	
	pwm_set_duty1(75);
	pwm_set_duty2(75);
	pwm_set_duty3(75);
	pwm_set_duty4(75);
	pwm_set_duty5(75);
	pwm_set_duty6(75);
	
	while (1);
	
	return 0;
}
