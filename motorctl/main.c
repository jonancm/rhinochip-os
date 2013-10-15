#include <p30fxxxx.h>

//_FWDT(WDT_OFF); // Turn off watchdog timer

//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers, BOR characteristics etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.
_FOSC(CSW_FSCM_OFF & XT_PLL8);  //Run this project using an external crystal
                                //routed via the PLL in 8x multiplier mode
                                //For the 7.3728 MHz crystal we will derive a
                                //throughput of 7.3728e+6*8/4 = 14.74 MIPS(Fcy)
                                //,~67nanoseconds instruction cycle time(Tcy).
_FWDT(WDT_OFF);                 //Turn off the Watch-Dog Timer.
_FBORPOR(MCLR_EN & PWRT_OFF);   //Enable MCLR reset pin and turn off the
                                //power-up timers.
_FGS(CODE_PROT_OFF);            //Disable Code Protection

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
