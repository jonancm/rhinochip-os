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
	
	do {
		copied = hostcom_read_cmd(cmd_buf, CMD_BUF_SIZE, &full);
	} while (!(copied && full));
	// FIXME: if the command is not fully read, the data will be discarded from
	// the buffer and lost forever and ever!
	
	cmd_buf[copied - 1] = '\0';
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
	
	if (token_type == TOKEN_INT)
	{
	}
	else if (token_type == TOKEN_DEC)
	{
	}
	else if (token_type == TOKEN_LETTER)
	{
	}
	else if (token_type == TOKEN_STR)
	{
	}
	else
	{
		// error
		return -1;
	}
	
	param_info->present = true;
	param_info->type = token_type;
	param_info->value = token_value;
	next_token();
	
	return retval;
}

/**
 * Lexical parser: parses the characters from the input buffer to extract the
 * longest token that it can.
 */
void next_token(void)
{
	for (; cmd_buf_pos < CMD_BUF_SIZE; ++cmd_buf_pos)
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
				++cmd_buf_pos;
				token_type = TOKEN_DEC;
				token_value.decimal.int_part = token_value.integer.sign * token_value.integer.abs_value;
			}
			
			for (; cmd_buf_pos < CMD_BUF_SIZE &&
			       cmd_buf[cmd_buf_pos] >= '0' &&
			       cmd_buf[cmd_buf_pos] <= '9';
			     ++cmd_buf_pos)
			{
				token_value.decimal.dec_part = (token_value.decimal.dec_part * 10) +
				                               (cmd_buf[cmd_buf_pos] - '0');
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
		}
		// Other characters: error
		else
		{
			break;
		}
	}
}
