#include "gpcorecom.h"

#include "../mcuicom.h"
#include "motor_status.h"
#include "../hostcmdset.h"

#include "../debug.h"

// debug
#include "../macros.h"

/**
 * Enumeration of all accepted token types
 */
typedef enum {
	TOKEN_LETTER = 256, // letter in ASCII representation
	TOKEN_INT, // integer number
	TOKEN_DEC, // decimal number (fixed point number with two decimal positions)
	TOKEN_STR, // string
	TOKEN_CMDEND, // command end mark
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
char cmd_buf[CMD_BUF_SIZE];

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

char cmd_name[2];
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
int next_token(void);
int cmd(void);
int instr(void);
int prog(void);
int param(param_t *param_info);
int num(void);
int letterparam(void);
int str(void);
void interpret_cmd(void);

inline void read_encoder_a(void);
inline void read_encoder_b(void);
inline void read_encoder_c(void);
inline void read_encoder_d(void);
inline void read_encoder_e(void);
inline void read_encoder_f(void);

inline void stop_motor_a(void);
inline void stop_motor_b(void);
inline void stop_motor_c(void);
inline void stop_motor_d(void);
inline void stop_motor_e(void);
inline void stop_motor_f(void);

inline void set_joint_abs_a(void);
inline void set_joint_abs_b(void);
inline void set_joint_abs_c(void);
inline void set_joint_abs_d(void);
inline void set_joint_abs_e(void);
inline void set_joint_abs_f(void);

inline void set_joint_rel_a(void);
inline void set_joint_rel_b(void);
inline void set_joint_rel_c(void);
inline void set_joint_rel_d(void);
inline void set_joint_rel_e(void);
inline void set_joint_rel_f(void);

inline void set_cartesian_abs_x(void);
inline void set_cartesian_abs_y(void);
inline void set_cartesian_abs_z(void);

inline void set_cartesian_rel_x(void);
inline void set_cartesian_rel_y(void);
inline void set_cartesian_rel_z(void);

inline void move_independent(void);
inline void move_coordinated(void);

/******************************************************************************
 *                           FUNCTION DEFINITIONS                             *
 ******************************************************************************/

void gpcorecom_interpret_next(void)
{
	bool_t full;
	int copied;
	
	dbgmsg_uart1("gpcorecom_interpret_next\n");
	
	if (mcuicom_cmd_available())
	{
		// Read the next command from the mcuicom buffer to the shell buffer
		copied = mcuicom_read_cmd(cmd_buf, CMD_BUF_SIZE, &full);
		cmd_buf_pos = 0;
		param1.present = false;
		param2.present = false;
		
		// Parse the command currently stored in the shell buffer
		parse_cmd();
	}
}

/**
 * Parse the command currently stored in the shell buffer.
 */
void parse_cmd(void)
{
	int retval;
	
	dbgmsg_uart1("parse_cmd\n");
	
	// Fetch the next token and parse it (lexical parser)
	next_token();
	
	// The command must be a single instruction...
	retval = instr();
	// If it's not an instruction, error
	if (retval < 0)
	{
		// error
	}
	// If it's an instruction, interpret it
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
	
	retval = cmd();
	if (retval > -1) // if there has been no error parsing the command
	{
		if (token_type == TOKEN_CMDEND) {}
		else if (token_type == TOKEN_COMMA)
		{
			next_token(); // Consume the comma and read the next token
			retval = param(&param1);
			if (retval > -1) // if there has been no error parsing the first parameter
			{
				if (token_type == TOKEN_CMDEND) {}
				else if (token_type == TOKEN_COMMA)
				{
					next_token(); // Consume the comma and read the next token
					retval = param(&param2);
				}
				else
					retval = -1; // syntax error
			}
		}
		else
			retval = -1; // syntax error
	}
	
	return retval;
}

/**
 * Non-terminal symbol 'prog'.
 */
int prog(void)
{
	int retval = -1;
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
			retval = -1;
		}
	}
	else
		retval = -1; // no command found
	
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
int next_token(void)
{
	int retval = 0;
	
	if (cmd_buf_pos < CMD_BUF_SIZE)
	{
		// Uppercase letters
		if (cmd_buf[cmd_buf_pos] >= 'A' && cmd_buf[cmd_buf_pos] <= 'Z')
		{
			token_type = TOKEN_LETTER;
			
			// Store token value
			token_value.letter = cmd_buf[cmd_buf_pos];
			
			++cmd_buf_pos;
		}
		// Lowercase letters
		else if (cmd_buf[cmd_buf_pos] >= 'a' && cmd_buf[cmd_buf_pos] <= 'z')
		{
			token_type = TOKEN_LETTER;
			
			// Store the token value and convert letter to upper case
			token_value.letter = cmd_buf[cmd_buf_pos] - ('a' - 'A');
			
			++cmd_buf_pos;
		}
		// Command (parameter separator)
		else if (cmd_buf[cmd_buf_pos] == ',')
		{
			token_type = TOKEN_COMMA;
			++cmd_buf_pos;
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
				++cmd_buf_pos;
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
			++cmd_buf_pos; // consume the opening quote
			for (str_length = 0;
			     cmd_buf_pos < CMD_BUF_SIZE &&
			     cmd_buf[cmd_buf_pos] != '"' &&
			     cmd_buf[cmd_buf_pos] != *CMDEND &&
			     str_length < MAX_STR_LENGTH;
			   ++cmd_buf_pos, ++str_length)
			{
				// store string literal
				token_value.string.charv[str_length] = cmd_buf[cmd_buf_pos];
			}
			token_value.string.length = str_length;
			
			// Error: the string is too long or hasn't been terminated with a
			// trailing quotation mark
			if (cmd_buf_pos >= CMD_BUF_SIZE || cmd_buf[cmd_buf_pos] == *CMDEND)
				retval = -1;
			
			++cmd_buf_pos;
		}
		// Line feed (end of command mark)
		else if (cmd_buf[cmd_buf_pos] == *CMDEND)
		{
			token_type = TOKEN_CMDEND;
			++cmd_buf_pos;
		}
		// Other characters: error
		else
		{
			// error
			retval = -1;
		}
	}
	
	return retval;
}

void interpret_cmd(void)
{
	dbgmsg_uart1("interpret_cmd\n");
	
	switch (cmd_name[0])
	{
		case 'A':
			switch (cmd_name[1])
			{
				// AA: Set joint absolute position of motor A
				case 'A':
					set_joint_abs_a(); break;
				// AB: Set joint absolute position of motor B
				case 'B':
					set_joint_abs_b(); break;
				// AC: Set joint absolute position of motor C
				case 'C':
					set_joint_abs_c(); break;
				// AD: Set joint absolute position of motor D
				case 'D':
					set_joint_abs_d(); break;
				// AE: Set joint absolute position of motor E
				case 'E':
					set_joint_abs_e(); break;
				// AF: Set joint absolute position of motor F
				case 'F':
					set_joint_abs_f(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'B':
			switch (cmd_name[1])
			{
				// BA: Set joint relative position of motor A
				case 'A':
					set_joint_rel_a(); break;
				// BB: Set joint relative position of motor B
				case 'B':
					set_joint_rel_b(); break;
				// BC: Set joint relative position of motor C
				case 'C':
					set_joint_rel_c(); break;
				// BD: Set joint relative position of motor D
				case 'D':
					set_joint_rel_d(); break;
				// BE: Set joint relative position of motor E
				case 'E':
					set_joint_rel_e(); break;
				// BF: Set joint relative position of motor F
				case 'F':
					set_joint_rel_f(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'C':
			switch (cmd_name[1])
			{
				// CX: Set cartesian absolute position of motor X
				case 'X':
					set_cartesian_abs_x(); break;
				// CY: Set cartesian absolute position of motor Y
				case 'Y':
					set_cartesian_abs_y(); break;
				// CZ: Set cartesian absolute position of motor Z
				case 'Z':
					set_cartesian_abs_z(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'D':
			switch (cmd_name[1])
			{
				// DX: Set cartesian relative position of motor X
				case 'X':
					set_cartesian_rel_x(); break;
				// DY: Set cartesian relative position of motor Y
				case 'Y':
					set_cartesian_rel_y(); break;
				// DZ: Set cartesian relative position of motor Z
				case 'Z':
					set_cartesian_rel_z(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'M':
			switch (cmd_name[1])
			{
				// MC: Move coordinated
				case 'C':
					move_coordinated(); break;
				// MI: Move independent
				case 'I':
					move_independent(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'R':
			switch (cmd_name[1])
			{
				// RA: Read encoder of motor A
				case 'A':
					read_encoder_a(); break;
				// RB: Read encoder of motor B
				case 'B':
					read_encoder_b(); break;
				// RC: Read encoder of motor C
				case 'C':
					read_encoder_c(); break;
				// RD: Read encoder of motor D
				case 'D':
					read_encoder_d(); break;
				// RE: Read encoder of motor E
				case 'E':
					read_encoder_e(); break;
				// RF: Read encoder of motor F
				case 'F':
					read_encoder_f(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		case 'S':
			switch (cmd_name[1])
			{
				// SA: Stop motor A
				case 'A':
					stop_motor_a(); break;
				// SB: Stop motor B
				case 'B':
					stop_motor_b(); break;
				// SC: Stop motor C
				case 'C':
					stop_motor_c(); break;
				// SD: Stop motor D
				case 'D':
					stop_motor_d(); break;
				// SE: Stop motor E
				case 'E':
					stop_motor_e(); break;
				// SF: Stop motor F
				case 'F':
					stop_motor_f(); break;
				default:
					// error: unknown command
					break;
			}
			break;
		default:
			// error: unknown command
			break;
	}
}

/******************************************************************************
 *                  MCUICOM COMMAND FUNCTION IMPLEMENTATION                   *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

inline void read_encoder_a(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_A], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void read_encoder_b(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_B], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void read_encoder_c(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_C], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void read_encoder_d(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_D], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void read_encoder_e(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_E], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void read_encoder_f(void)
{
	char buf[64];
	snprintf(buf, 64, "%d%c", motor_steps[MOTOR_F], *CMDEND);
	mcuicom_send(buf, strlen(buf));
}

inline void stop_motor_a(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void stop_motor_b(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void stop_motor_c(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void stop_motor_d(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void stop_motor_e(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void stop_motor_f(void)
{
	// TODO: implement
	
	// Set PWM level to zero, so that motor doesn't move
	
	// Set destination position to current position
}

inline void set_joint_abs_a(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_A] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_a: %d\n", motor_desired_pos[MOTOR_A]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_abs_b(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_B] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_b: %d\n", motor_desired_pos[MOTOR_B]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_abs_c(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_C] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_c: %d\n", motor_desired_pos[MOTOR_C]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_abs_d(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_D] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_d: %d\n", motor_desired_pos[MOTOR_D]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_abs_e(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_E] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_e: %d\n", motor_desired_pos[MOTOR_E]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_abs_f(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			motor_desired_pos[MOTOR_F] = intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_abs_f: %d\n", motor_desired_pos[MOTOR_F]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_a(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_A] = motor_steps[MOTOR_A] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_a: %d\n", motor_desired_pos[MOTOR_A]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_b(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_B] = motor_steps[MOTOR_B] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_b: %d\n", motor_desired_pos[MOTOR_B]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_c(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_C] = motor_steps[MOTOR_C] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_c: %d\n", motor_desired_pos[MOTOR_C]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_d(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_D] = motor_steps[MOTOR_D] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_d: %d\n", motor_desired_pos[MOTOR_D]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_e(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_E] = motor_steps[MOTOR_E] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_e: %d\n", motor_desired_pos[MOTOR_E]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_joint_rel_f(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			// A relative position increment means: desired position = current position + increment
			motor_desired_pos[MOTOR_F] = motor_steps[MOTOR_F] + intparam1;
			
			#ifndef NDEBUG
			char buf[64];
			snprintf(buf, 64, "set_joint_rel_f: %d\n", motor_desired_pos[MOTOR_F]);
			dbgmsg_uart1(buf);
			#endif
		}
		else
		{
			// error: parameter must be an integer number
		}
	}
	else
	{
		// error: parameter must be specified
	}
}

inline void set_cartesian_abs_x(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_abs_x: %f\n", cartesian_desired_pos[COORD_X]);
	mcuicom_send(buf, strlen(buf));
}

inline void set_cartesian_abs_y(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_abs_y: %f\n", cartesian_desired_pos[COORD_Y]);
	mcuicom_send(buf, strlen(buf));
}

inline void set_cartesian_abs_z(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_abs_z: %f\n", cartesian_desired_pos[COORD_Z]);
	mcuicom_send(buf, strlen(buf));
}

inline void set_cartesian_rel_x(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_rel_x: %f\n", cartesian_desired_pos[COORD_X]);
	mcuicom_send(buf, strlen(buf));
}

inline void set_cartesian_rel_y(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_rel_y: %f\n", cartesian_desired_pos[COORD_Y]);
	mcuicom_send(buf, strlen(buf));
}

inline void set_cartesian_rel_z(void)
{
	// TODO: implement
	char buf[64];
	snprintf(buf, 64, "set_cartesian_rel_z: %f\n", cartesian_desired_pos[COORD_Z]);
	mcuicom_send(buf, strlen(buf));
}

inline void move_independent(void)
{
	// TODO: implement properly (the code below is just for testing PD and PR)
	motor_steps[MOTOR_A] = motor_desired_pos[MOTOR_A];
	motor_steps[MOTOR_B] = motor_desired_pos[MOTOR_B];
	motor_steps[MOTOR_C] = motor_desired_pos[MOTOR_C];
	motor_steps[MOTOR_D] = motor_desired_pos[MOTOR_D];
	motor_steps[MOTOR_E] = motor_desired_pos[MOTOR_E];
	motor_steps[MOTOR_F] = motor_desired_pos[MOTOR_F];
	dbgmsg_uart1("move_independent\n");
}

inline void move_coordinated(void)
{
	// TODO: implement properly (the code below is just for testing PD and PR)
	motor_steps[MOTOR_A] = motor_desired_pos[MOTOR_A];
	motor_steps[MOTOR_B] = motor_desired_pos[MOTOR_B];
	motor_steps[MOTOR_C] = motor_desired_pos[MOTOR_C];
	motor_steps[MOTOR_D] = motor_desired_pos[MOTOR_D];
	motor_steps[MOTOR_E] = motor_desired_pos[MOTOR_E];
	motor_steps[MOTOR_F] = motor_desired_pos[MOTOR_F];
	dbgmsg_uart1("move_coordinated\n");
}
