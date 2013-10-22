#include <p30fxxxx.h>

// Configuration settings
_FOSC(CSW_FSCM_OFF & FRC_PLL16); // Fosc = 16x 8 MHz, Fcy = 32 MHz
_FWDT(WDT_OFF);                  // Watchdog timer off
_FBORPOR(MCLR_DIS);              // Disable reset pin

#include "../delay.h"
#include "pwm.h"
#include "qei.h"


int main(void)
{
	
	qei_setup();
	
	pwm_setup();
	pwm_set_pdc1(75);
	
	while (1)
	{
		Delay5ms(100);
	}
	
	return 0;
}
