#include "shell.h"
#include "hostcom.h"
#include "../types.h"

/**
 * Enumeration of all accepted token types
 */
typedef enum {
	TOKEN_LETTER = 256, // letter in ASCII representation
	TOKEN_INT, // integer number
	TOKEN_DEC, // decimal number (fixed point number with two decimal positions)
	TOKEN_STR, // string
	TOKEN_LF, // line feed (command end mark)
	TOKEN_COMMA // parameter separator
} token_type_t;

#define MAX_STR_LENGTH    64

/**
 * Union to store the value of the token
 */
typedef union {
	char letter;
	
	/**
	 * If the token is an integer, store its sing and absolute value in a struct
	 */
	struct {
		int sign;
		int abs_value;
	} integer;
	
	/**
	 * If the token is a decimal number, store its integer part and its decimal
	 * part in a struct
	 */
	struct {
		int int_part; // integer part
		int dec_part; // decimal part
	} decimal;
	
	struct {
		char charv[MAX_STR_LENGTH]; // character vector
		int length;
	} string;
} token_value_t;

/**
 * Buffer where received commands are stored for the parser to read them.
 */
#define CMD_BUF_SIZE    128
byte_t cmd_buf[CMD_BUF_SIZE];

/**
 * Position in the command buffer where the lexical analyzer is parsing.
 */
int cmd_buf_pos = 0;

token_type_t     token_type;
token_value_t    token_value;

/**
 * Flag that indicates whether the shell is running in interactive mode or in
 * program mode (running a program).
 */
bool_t interactive;

byte_t cmd_name[2];
typedef struct {
	bool_t           present; // whether the parameter has been specified or not
	token_type_t     type;    // type of the parameter
	token_value_t    value;   // value of the parameter
} param_t;
param_t param1;
param_t param2;

/******************************************************************************
 *                           FUNCTION DECLARATIONS                            *
 ******************************************************************************/

void parse_cmd(void);
void next_cmd(void);
void next_token(void);
int cmd(void);
int instr(void);
int prog(void);
int param(param_t *param_info);
int num(void);
int letterparam(void);
int str(void);
void interpret_cmd(void);

/******************************************************************************
 *                           FUNCTION DEFINITIONS                             *
 ******************************************************************************/

void shell_run_interactive(void)
{
	interactive = true;
	
	while (1)
	{
		// Read the next command from the hostcom buffer to the shell buffer
		next_cmd();
		// Parse the command currently stored in the shell buffer
		parse_cmd();
	}
}

/**
 * Read the next command from the hostcom buffer and place it into the shell
 * buffer (i.e. the buffer that the shell uses to store the symbols that are
 * being parsed).
 */
void next_cmd(void)
{
	bool_t full;
	int copied;
	
	while (!hostcom_cmd_available());
	copied = hostcom_read_cmd(cmd_buf, CMD_BUF_SIZE, &full);
}

/**
 * Parse the command currently stored in the shell buffer.
 */
void parse_cmd(void)
{
	int retval;
	
	// Fetch the next token and parse it (lexical parser)
	next_token();
	
	// The command can be a single instruction...
	retval = instr();
	if (retval < 0) // it's not an instruction
	{
		// ... or a program
		retval = prog();
		if (retval < 0) // it's not a program
		{
			// Syntax error (it's not an instruction nor a program)
		}
	}
	else
	{
		interpret_cmd();
	}
}

/**
 * Non-terminal symbol 'instr'.
 */
int instr(void)
{
	int retval = 0;
	
	cmd();
	if (token_type == TOKEN_LF) {}
	else if (token_type == TOKEN_COMMA)
	{
		next_token(); // Consume the comma and read the next token
		param(&param1);
		if (token_type == TOKEN_LF) {}
		else if (token_type == TOKEN_COMMA)
		{
			next_token(); // Consume the comma and read the next token
			param(&param2);
		}
		else
			retval = -1; // syntax error
	}
	else
		retval = -1; // syntax error
	
	return retval;
}

/**
 * Non-terminal symbol 'prog'.
 */
int prog(void)
{
	int retval = 0;
	return retval;
}

/**
 * Non-terminal symbol 'cmd'.
 */
int cmd(void)
{
	int retval = 0;
	
	if (token_type == TOKEN_LETTER)
	{
		cmd_name[0] = token_value.letter;
		next_token();
		if (token_type == TOKEN_LETTER)
		{
			cmd_name[1] = token_value.letter;
			next_token();
		}
		else
		{
			// Syntax error
		}
	}
	else
	{
		// Syntax error?
	}
	
	return retval;
}

/**
 * Non-terminal symbol 'param'.
 */
int param(param_t *param_info)
{
	int retval = 0;
	
	if (token_type != TOKEN_INT &&
	    token_type != TOKEN_DEC &&
	    token_type != TOKEN_LETTER &&
	    token_type != TOKEN_STR)
	{
		// error
		retval = -1;
	}
	else
	{
		param_info->present = true;
		param_info->type = token_type;
		param_info->value = token_value;
		next_token();
	}
	
	return retval;
}

/**
 * Lexical parser: parses the characters from the input buffer to extract the
 * longest token that it can.
 */
void next_token(void)
{
	if (cmd_buf_pos < CMD_BUF_SIZE)
	{
		// Uppercase letters
		if (cmd_buf[cmd_buf_pos] >= 'A' && cmd_buf[cmd_buf_pos] <= 'Z')
		{
			token_type = TOKEN_LETTER;
			
			// Store token value
			token_value.letter = cmd_buf[cmd_buf_pos];
		}
		// Lowercase letters
		else if (cmd_buf[cmd_buf_pos] >= 'a' && cmd_buf[cmd_buf_pos] <= 'z')
		{
			token_type = TOKEN_LETTER;
			
			// Store the token value and convert letter to upper case
			token_value.letter = cmd_buf[cmd_buf_pos] - ('a' - 'A');
		}
		// Command (parameter separator)
		else if (cmd_buf[cmd_buf_pos] == ',')
		{
			token_type = TOKEN_COMMA;
		}
		// Numbers: -?[0-9]+(.[0-9]+)
		else if (cmd_buf[cmd_buf_pos] == '-' || (cmd_buf[cmd_buf_pos] >= '0' && cmd_buf[cmd_buf_pos] <= '9'))
		{
			token_type = TOKEN_INT;
			token_value.integer.sign = 1;
			token_value.integer.abs_value = 0;
			
			if (cmd_buf[cmd_buf_pos] == '-')
			{
				token_value.integer.sign = -1;
			}
			
			for (; cmd_buf_pos < CMD_BUF_SIZE &&
			       cmd_buf[cmd_buf_pos] >= '0' &&
			       cmd_buf[cmd_buf_pos] <= '9';
			     ++cmd_buf_pos)
			{
				token_value.integer.abs_value = (token_value.integer.abs_value * 10) +
				                                (cmd_buf[cmd_buf_pos] - '0');
			}
			
			if (cmd_buf_pos < CMD_BUF_SIZE && cmd_buf[cmd_buf_pos] == '.')
			{
				int num_dec_digits; // number of decimal digits
				
				++cmd_buf_pos;
				token_type = TOKEN_DEC;
				token_value.decimal.int_part = token_value.integer.sign * token_value.integer.abs_value;
				
				for (num_dec_digits = 0;
				     cmd_buf_pos < CMD_BUF_SIZE &&
				     num_dec_digits < 2 &&
				     cmd_buf[cmd_buf_pos] >= '0' &&
				     cmd_buf[cmd_buf_pos] <= '9';
				   ++cmd_buf_pos, ++num_dec_digits)
				{
					token_value.decimal.dec_part = (token_value.decimal.dec_part * 10) +
					                               (cmd_buf[cmd_buf_pos] - '0');
				}
			}
		}
		// String literals
		else if (cmd_buf[cmd_buf_pos] == '"')
		{
			int str_length;
			token_type = TOKEN_STR;
			for (str_length = 0;
			     cmd_buf_pos < CMD_BUF_SIZE &&
			     cmd_buf[cmd_buf_pos] != '"' &&
			     str_length < MAX_STR_LENGTH;
			   ++cmd_buf_pos, ++str_length)
			{
				// store string literal
				token_value.string.charv[str_length] = cmd_buf[cmd_buf_pos];
			}
			token_value.string.length = str_length;
		}
		// Line feed (end of command mark)
		else if (cmd_buf[cmd_buf_pos] == '\n')
		{
			token_type = TOKEN_LF;
			++cmd_buf_pos;
		}
		// Other characters: error
		else
		{
			// error
		}
	}
}

void interpret_cmd(void)
{
	switch (cmd_name[0])
	{
		case 'A':
			switch (cmd_name[1])
			{
				// AR: Read System Acceleration
				case 'R':
					break;
				// AC: Clear Actual Position
				case 'C':
					break;
				// AS: Set System Acceleration
				case 'S':
					break;
				default:
					break;
			}
			break;
		case 'C':
			switch (cmd_name[1])
			{
				// CC: Set Coordinate Position
				case 'C':
					break;
				// CG: Enable/Disable Gripper Mode
				case 'G':
					break;
				// CM: Set Motor Mode
				case 'M':
					break;
				// CR: Set Robot Type
				case 'R':
					break;
				default:
					break;
			}
			break;
		case 'D':
			switch (cmd_name[1])
			{
				// DR: Read Motor PWM Level and Direction
				case 'R':
					break;
				// DS: Set PWM Level and Direction
				case 'S':
					break;
				default:
					break;
			}
			break;
		case 'F':
			switch (cmd_name[1])
			{
				// FR: Receive Teach Pendant File from Host
				case 'R':
					break;
				// FT: Transmit Teach Pendant File to Host
				case 'T':
					break;
				// FX: Execute Teach Pendant Program
				case 'X':
					break;
				default:
					break;
			}
			break;
		case 'G':
			switch (cmd_name[1])
			{
				// GS: Read Gripper Status
				case 'S':
					break;
				// GC: Close Gripper
				case 'C':
					break;
				// GO: Open Gripper
				case 'O':
					break;
				default:
					break;
			}
			break;
		case 'H':
			switch (cmd_name[1])
			{
				// HR: Read Soft Home Position
				case 'R':
					break;
				// HA: Got To the Hard Home Position
				case 'A':
					break;
				// HG: Go To the Soft Home Position
				case 'G':
					break;
				// HH: Execute a Hard Home
				case 'H':
					break;
				// HL: Hard Home on Limit Switch
				case 'L':
					break;
				// HS: Set Soft Home
				case 'S':
					break;
				default:
					break;
			}
			break;
		case 'I':
			switch (cmd_name[1])
			{
				// IB: Read Input or Swicht Bit
				case 'B':
					break;
				// IP: Read Input Port
				case 'P':
					break;
				// IX: Read Switch Port
				case 'X':
					break;
				default:
					break;
			}
			break;
		case 'K':
			switch (cmd_name[1])
			{
				// KA: Set Proportional Gain
				case 'A':
					break;
				// KB: Set Differential Gain
				case 'B':
					break;
				// KC: Set Integral Gain
				case 'C':
					break;
				// KR: Restore User Gains from EEPROM
				case 'R':
					break;
				// KS: Store User Gains to EEPROM
				case 'S':
					break;
				// KX: Restore Factory Gains
				case 'X':
					break;
				default:
					break;
			}
			break;
		case 'M':
			switch (cmd_name[1])
			{
				// MA: Stop All Motors and Aux Ports
				case 'A':
					break;
				// MC: Start Coordinated Move
				case 'C':
					break;
				// MI: Start All Motors, Immediate Mode
				case 'I':
					break;
				// MM: Stop Single Motor
				case 'M':
					break;
				// MS: Start Single Motor
				case 'S':
					break;
				// MX: Start an XYZ Move
				case 'X':
					break;
				default:
					break;
			}
			break;
		case 'O':
			switch (cmd_name[1])
			{
				// OB: Set Output Bit
				case 'B':
					break;
				// OP: Set Output Port
				case 'P':
					break;
				// OR: Read Output Port
				case 'R':
					break;
				// OT: Toggle Output Bit
				case 'T':
					break;
				default:
					break;
			}
			break;
		case 'P':
			switch (cmd_name[1])
			{
				// PA: Read Actual Position
				case 'A':
					break;
				// PW: Read Destination Position
				case 'W':
					break;
				// PZ: Read XYZ Destination Position
				case 'Z':
					break;
				// PD: Set Destination Position, Absolute
				case 'D':
					break;
				// PR: Set Destination Position, Relative
				case 'R':
					break;
				// PX: Set XYZ Destination, Absolute
				case 'X':
					break;
				// PY Set XYZ Destination, Relative
				case 'Y':
					break;
				default:
					break;
			}
			break;
		case 'R':
			switch (cmd_name[1])
			{
				// RL: Read Limit Switches
				case 'L':
					break;
				// RA: Read Proportional Gain
				case 'A':
					break;
				// RB: Read Differential Gain
				case 'B':
					break;
				// RC: Read Integral Gain
				case 'C':
					break;
				default:
					break;
			}
			break;
		case 'S':
			switch (cmd_name[1])
			{
				// SA: Read Motor Status
				case 'A':
					break;
				// SC: Read System Configuration
				case 'C':
					break;
				// SD: Stop/Start Delay Timer
				case 'D':
					break;
				// SE: Read Host Error Stack
				case 'E':
					break;
				// SM: Read Motor Mode
				case 'M':
					break;
				// SP: Read Teach Pendant Error Byte
				case 'P':
					break;
				// SR: Reset Motor Current Limit Circuitry
				case 'R':
					break;
				// SS: Read System Status
				case 'S':
					break;
				// ST: Execute Diagnostics
				case 'T':
					break;
				// SU: Read Usage Time
				case 'U':
					break;
				// SV: Read Version and I.D. Number
				case 'V':
					break;
				// SX: Execute Diagnostics and Return Results
				case 'X':
					break;
				// SZ: Read the Delay Timer Value
				case 'Z':
					break;
				default:
					break;
			}
			break;
		case 'T':
			switch (cmd_name[1])
			{
				// TA: Abort/Terminate Teach Pendant Program
				case 'A':
					break;
				// TC: Clear Teach Pendant Display
				case 'C':
					break;
				// TD: Print to Teach Pendant Display
				case 'D':
					break;
				// TE: Enable/Disable Teach Pendant to Move Motors
				case 'E':
					break;
				// TH: Give Control to Host
				case 'H':
					break;
				// TX: Give Control to Teach Pendant
				case 'X':
					break;
				// TK: Return to Host the Next Key Code
				case 'K':
					break;
				// TL: Return to Host the Last Key Code
				case 'L':
					break;
				// TR: Reset the Teach Pendant
				case 'R':
					break;
				// TS: Set Teach Pendant Display Cursor
				case 'S':
					break;
				// TT: Execute Teach Pendant Diagnostics and Return Results
				case 'T':
					break;
				default:
					break;
			}
			break;
		case 'U':
			switch (cmd_name[1])
			{
				// UA: Read XYZ Rotation Angle
				case 'A':
					break;
				// UH: Read XYZ Home Position
				case 'H':
					break;
				// UO: Read XYZ Offset
				case 'O':
					break;
				// UT: Read Tool Length
				case 'T':
					break;
				// UY: Read Height of Elbow Rotation Axis
				case 'Y':
					break;
				default:
					break;
			}
			break;
		case 'V':
			switch (cmd_name[1])
			{
				// VA: Read Motor Actual Velocity
				case 'A':
					break;
				// VR: Read Motor Desired Velocity
				case 'R':
					break;
				// VX: Read System Velocity
				case 'X':
					break;
				// VG: Set System Velocity
				case 'G':
					break;
				// VS: Set Motor Velocity
				case 'S':
					break;
				default:
					break;
			}
			break;
		case 'W':
			switch (cmd_name[1])
			{
				// WA: Abort all Waits
				case 'A':
					break;
				// WI: Wait on Input or Switch
				case 'I':
					break;
				default:
					break;
			}
			break;
		case 'X':
			switch (cmd_name[1])
			{
				// XR: Read Auxiliary Port Level and Direction
				case 'R':
					break;
				// XA: Set XYZ Rotation Angle
				case 'A':
					break;
				// XH: Set XYZ Home Position
				case 'H':
					break;
				// XO: Set XYZ Offset
				case 'O':
					break;
				// XS: Set Aux Port Level and Direction
				case 'S':
					break;
				// XT: Set Tool Length
				case 'T':
					break;
				// XY: Set Height of Elbow Rotation Axis
				case 'Y':
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
}
