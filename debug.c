#include "debug.h"

#ifndef NDEBUG

#include <p30fxxxx.h>

void dbgmsg_uart1(char *c)
{
	for (; *c != 0; ++c)
	{
		while (U1STAbits.UTXBF);
		U1TXREG = *c;
	}
}

void dbgmsg_uart2(char *c)
{
	for (; *c != 0; ++c)
	{
		while (U2STAbits.UTXBF);
		U2TXREG = *c;
	}
}

#endif
