#include "mctlcom.h"

#include "../debug.h"

int mctlcom_get_response(char *response, int size)
{
	bool_t    full;
	int       copied = 0;
	
	// Wait until the receive buffer has at least one full command
	while (!mcuicom_cmd_available());
	
	// Otherwise, if a command has been fully received, get it and extract the response code
	// in order to return it to the callee
	copied = mcuicom_read_cmd(response, size, &full);
	#ifndef NDEBUG
	dbgmsg_uart2("response received\n");
	#endif
	
	return copied;
}
