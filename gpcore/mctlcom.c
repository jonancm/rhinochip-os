#include "mctlcom.h"

int mctlcom_get_response(void)
{
	mcuicom_cmd    cmd;
	int            i, j;
	const int      recvd = 5;
	
	/*
	while (!mcuicom_rcv_buf.used);
	cmd.opcode = mcuicom_rcv_buf.data[0];
	
	int recvd, cmd_size = sizeof(cmd.opcode) + mcuicom_param_size(&cmd);
	// Wait until all of the bytes of the command have been received
	while (mcuicom_rcv_buf.used < cmd_size);
	// Read the command from the buffer
	for (recvd = 1; recvd < cmd_size; ++recvd)
		cmd.data[recvd] = mcuicom_rcv_buf.data[recvd];
	
	// Disable UART 1 receive interrupts to prevent CPU from messing
	// with the 'mcuicom_rcv_buf' buffer
	IEC0bits.U1RXIE = 0;
	
	int i, j;
	// Shift data in the buffer to remove consumed command
	for (i = 0, j = recvd; j < mcuicom_rcv_buf.used; ++i, ++j)
		mcuicom_rcv_buf.data[i] = mcuicom_rcv_buf.data[j];
	mcuicom_rcv_buf.used -= recvd;
	
	// Re-enable UART 1 receive interrupts
	IEC0bits.U1RXIE = 1;
	*/
	
	while (mcuicom_rcv_buf.used < 5);
	cmd.data[0] = mcuicom_rcv_buf.data[0]; // opcode
	cmd.data[1] = mcuicom_rcv_buf.data[1]; // param 0 (byte 3)
	cmd.data[2] = mcuicom_rcv_buf.data[2]; // param 0 (byte 2)
	cmd.data[3] = mcuicom_rcv_buf.data[3]; // param 0 (byte 1)
	cmd.data[4] = mcuicom_rcv_buf.data[4]; // param 0 (byte 0)
	/*
	TODO:
	cmd = *mcuicom_rcv_buf.data;
	*/
	
	// Disable UART 1 receive interrupts to prevent CPU from messing
	// with the 'mcuicom_rcv_buf' buffer
	IEC0bits.U1RXIE = 0;
	
	// Shift data in the buffer to remove consumed command
	for (i = 0, j = recvd; j < mcuicom_rcv_buf.used; ++i, ++j)
		mcuicom_rcv_buf.data[i] = mcuicom_rcv_buf.data[j];
	mcuicom_rcv_buf.used -= recvd;
	
	// Re-enable UART 1 receive interrupts
	IEC0bits.U1RXIE = 1;
	
	return cmd.param[0];
}
