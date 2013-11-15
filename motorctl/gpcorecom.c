#include "gpcorecom.h"

#include "../mcuicom.h"
#include "motor_status.h"

/**
 * Temporary variable to store the next command that is being parsed from the receive buffer.
 */
mcuicom_cmd    next_cmd;

/**
 * Index of the next data byte of the command to be received. This points to the position of
 * the command data buffer where the next received byte should be stored.
 */
int            next_cmd_data_pos = 0;

/**
 * Size of the next command to be received, in bytes. This is the sum of the size of the opcode
 * plus the size of the parameters.
 */
int            next_cmd_size = 1;

void gpcorecom_interpret_next(void)
{
	int            i, j;
	mcuicom_cmd    response;
	bool_t         cmd_full = false;
	
	// Operate on the buffer's data only if it actually contains data
	if (mcuicom_rcv_buf.used)
	{
		// Extract the available bytes of next command from the receive buffer
		if (next_cmd_data_pos == 0)
		{
			next_cmd.data[next_cmd_data_pos++] = mcuicom_rcv_buf.data[0];
			next_cmd_size += mcuicom_param_size(&next_cmd);
		}
		for (i = next_cmd_data_pos; i < mcuicom_rcv_buf.used && i < next_cmd_size; ++i)
			next_cmd.data[next_cmd_data_pos] = mcuicom_rcv_buf.data[i];
		// If the command has not been fully received, update 'next_cmd_data_pos'.
		// Otherwise, reset 'next_cmd_data_pos' and 'next_cmd_size' to be able to process the next command.
		if (i < next_cmd_size)
			next_cmd_data_pos = i;
		else
		{
			next_cmd_data_pos = 0;
			next_cmd_size = 1;
			cmd_full = true;
		}
		
		// Disable UART1 receive interrupts, to prevent CPU from interrupting
		// while modifying the receive buffer
		IEC0bits.U1RXIE = 0;
		// Shift data in the receive buffer to remove already processed bytes
		for (j = 0; i < mcuicom_rcv_buf.used; ++i, ++j)
			mcuicom_rcv_buf.data[j] = mcuicom_rcv_buf.data[i];
		mcuicom_rcv_buf.used -= j;
		// Re-enable UART1 receive interrupts
		IEC0bits.U1RXIE = 1;
		
		// Interpret command
		if (cmd_full)
		{
			response.opcode = MCUICOM_RESPONSE;
			switch (next_cmd.opcode)
			{
				case MCUICOM_READ_ENC_MA: response.param[0] = motor_steps[MOTOR_A]; mcuicom_send(&response); break;
				case MCUICOM_READ_ENC_MB: response.param[0] = motor_steps[MOTOR_B]; mcuicom_send(&response); break;
				case MCUICOM_READ_ENC_MC: response.param[0] = motor_steps[MOTOR_C]; mcuicom_send(&response); break;
				case MCUICOM_READ_ENC_MD: response.param[0] = motor_steps[MOTOR_D]; mcuicom_send(&response); break;
				case MCUICOM_READ_ENC_ME: response.param[0] = motor_steps[MOTOR_E]; mcuicom_send(&response); break;
				case MCUICOM_READ_ENC_MF: response.param[0] = motor_steps[MOTOR_F]; mcuicom_send(&response); break;
			}
		}
	}
}
