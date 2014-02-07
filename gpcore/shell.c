#include "shell.h"
#include "hostcom.h"
#include "../types.h"
#include "../hostcmdset.h"
#include "controller_status.h"
#include "self_test.h"
#include "../mcuicom.h"
#include "mctlcom.h"
#include "hardhome.h"

#include <stdlib.h> // atoi

#include "../debug.h"
//#ifndef NDEBUG
#define CMD_RECVD "Command received: "
#define ERROR "Error\n"
#define ERR_UNKOWN_CMD             "Error: unknown command\n"
#define ERR_OUT_OF_RANGE           "Error: parameter out of range\n"
#define ERR_MISSING_PARAMS         "Error: wrong number of parameters\n"
#define ERR_TRAPEZOIDAL_MOVE       "Error: motor still executing a trapezoidal move\n"
#define ERR_WRONG_TYPE_PARAM       "Error: parameter of wrong type has been provided\n"
#define ERR_NOT_ROBOT_MODE         "Error: controller is not in robot mode\n"
#define ERR_NO_HARD_HOME           "Error: a hard home has not been executed yet\n"
#define ERR_EXECUTING_HARD_HOME    "Error: a hard home is in progress\n"
#define ERR_TEACH_PENDANT_MODE     "Error: command not allowed in teach pendant mode\n"
#define ERR_GRIPPER_NOT_ENABLED    "Error: the gripper is not enabled\n"
#define ERR_PORT_A_IS_GRIPPER      "Error: port A is enabled as the gripper\n"
//#endif

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

inline void hostcmd_sa(void);
inline void hostcmd_sc(void);
inline void hostcmd_sd(void);
inline void hostcmd_se(void);
inline void hostcmd_sm(void);
inline void hostcmd_sp(void);
inline void hostcmd_sr(void);
inline void hostcmd_ss(void);
inline void hostcmd_st(void);
inline void hostcmd_su(void);
inline void hostcmd_sv(void);
inline void hostcmd_sx(void);
inline void hostcmd_sz(void);
inline void hostcmd_cc(void);
inline void hostcmd_cg(void);
inline void hostcmd_cm(void);
inline void hostcmd_cr(void);
inline void hostcmd_ar(void);
inline void hostcmd_dr(void);
inline void hostcmd_gs(void);
inline void hostcmd_hr(void);
inline void hostcmd_pa(void);
inline void hostcmd_pw(void);
inline void hostcmd_pz(void);
inline void hostcmd_rl(void);
inline void hostcmd_ua(void);
inline void hostcmd_uh(void);
inline void hostcmd_uo(void);
inline void hostcmd_ut(void);
inline void hostcmd_uy(void);
inline void hostcmd_va(void);
inline void hostcmd_vr(void);
inline void hostcmd_vx(void);
inline void hostcmd_xr(void);
inline void hostcmd_ac(void);
inline void hostcmd_as(void);
inline void hostcmd_ds(void);
inline void hostcmd_gc(void);
inline void hostcmd_go(void);
inline void hostcmd_ha(void);
inline void hostcmd_hg(void);
inline void hostcmd_hh(void);
inline void hostcmd_hl(void);
inline void hostcmd_hs(void);
inline void hostcmd_ma(void);
inline void hostcmd_mc(void);
inline void hostcmd_mi(void);
inline void hostcmd_mm(void);
inline void hostcmd_ms(void);
inline void hostcmd_mx(void);
inline void hostcmd_pd(void);
inline void hostcmd_pr(void);
inline void hostcmd_px(void);
inline void hostcmd_py(void);
inline void hostcmd_vg(void);
inline void hostcmd_vs(void);
inline void hostcmd_xa(void);
inline void hostcmd_xh(void);
inline void hostcmd_xo(void);
inline void hostcmd_xs(void);
inline void hostcmd_xt(void);
inline void hostcmd_xy(void);
inline void hostcmd_fr(void);
inline void hostcmd_ft(void);
inline void hostcmd_fx(void);
inline void hostcmd_ta(void);
inline void hostcmd_tc(void);
inline void hostcmd_td(void);
inline void hostcmd_te(void);
inline void hostcmd_th(void);
inline void hostcmd_tx(void);
inline void hostcmd_tk(void);
inline void hostcmd_tl(void);
inline void hostcmd_tr(void);
inline void hostcmd_ts(void);
inline void hostcmd_tt(void);
inline void hostcmd_ka(void);
inline void hostcmd_kb(void);
inline void hostcmd_kc(void);
inline void hostcmd_ra(void);
inline void hostcmd_rb(void);
inline void hostcmd_rc(void);
inline void hostcmd_kr(void);
inline void hostcmd_ks(void);
inline void hostcmd_kx(void);
inline void hostcmd_ib(void);
inline void hostcmd_ip(void);
inline void hostcmd_ix(void);
inline void hostcmd_ob(void);
inline void hostcmd_op(void);
inline void hostcmd_or(void);
inline void hostcmd_ot(void);
inline void hostcmd_wa(void);
inline void hostcmd_wi(void);

/******************************************************************************
 *                           FUNCTION DEFINITIONS                             *
 ******************************************************************************/

void shell_run_interactive(void)
{
	interactive = true;
	
	// Send version information to host PC upon controller startup,
	// as though the SV command had been sent by the host PC.
	hostcmd_sv();
	
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
	cmd_buf_pos = 0;
	param1.present = false;
	param2.present = false;
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
			dbgmsg_uart2(ERROR);
		}
		else
		{
			// save program to EEPROM
		}
	}
	else // it's an instruction
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
	switch (cmd_name[0])
	{
		case 'A':
			switch (cmd_name[1])
			{
				// AR: Read System Acceleration
				case 'R':
					hostcmd_ar(); break;
				// AC: Clear Actual Position
				case 'C':
					hostcmd_ac(); break;
				// AS: Set System Acceleration
				case 'S':
					hostcmd_as(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'C':
			switch (cmd_name[1])
			{
				// CC: Set Coordinate Position
				case 'C':
					hostcmd_cc(); break;
				// CG: Enable/Disable Gripper Mode
				case 'G':
					hostcmd_cg(); break;
				// CM: Set Motor Mode
				case 'M':
					hostcmd_cm(); break;
				// CR: Set Robot Type
				case 'R':
					hostcmd_cr(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'D':
			switch (cmd_name[1])
			{
				// DR: Read Motor PWM Level and Direction
				case 'R':
					hostcmd_dr(); break;
				// DS: Set PWM Level and Direction
				case 'S':
					hostcmd_ds(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'F':
			switch (cmd_name[1])
			{
				// FR: Receive Teach Pendant File from Host
				case 'R':
					hostcmd_fr(); break;
				// FT: Transmit Teach Pendant File to Host
				case 'T':
					hostcmd_ft(); break;
				// FX: Execute Teach Pendant Program
				case 'X':
					hostcmd_fx(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'G':
			switch (cmd_name[1])
			{
				// GS: Read Gripper Status
				case 'S':
					hostcmd_gs(); break;
				// GC: Close Gripper
				case 'C':
					hostcmd_gc(); break;
				// GO: Open Gripper
				case 'O':
					hostcmd_go(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'H':
			switch (cmd_name[1])
			{
				// HR: Read Soft Home Position
				case 'R':
					hostcmd_hr(); break;
				// HA: Got To the Hard Home Position
				case 'A':
					hostcmd_ha(); break;
				// HG: Go To the Soft Home Position
				case 'G':
					hostcmd_hg(); break;
				// HH: Execute a Hard Home
				case 'H':
					hostcmd_hh(); break;
				// HL: Hard Home on Limit Switch
				case 'L':
					hostcmd_hl(); break;
				// HS: Set Soft Home
				case 'S':
					hostcmd_hs(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'I':
			switch (cmd_name[1])
			{
				// IB: Read Input or Swicht Bit
				case 'B':
					hostcmd_ib(); break;
				// IP: Read Input Port
				case 'P':
					hostcmd_ip(); break;
				// IX: Read Switch Port
				case 'X':
					hostcmd_ix(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'K':
			switch (cmd_name[1])
			{
				// KA: Set Proportional Gain
				case 'A':
					hostcmd_ka(); break;
				// KB: Set Differential Gain
				case 'B':
					hostcmd_kb(); break;
				// KC: Set Integral Gain
				case 'C':
					hostcmd_kc(); break;
				// KR: Restore User Gains from EEPROM
				case 'R':
					hostcmd_kr(); break;
				// KS: Store User Gains to EEPROM
				case 'S':
					hostcmd_ks(); break;
				// KX: Restore Factory Gains
				case 'X':
					hostcmd_kx(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'M':
			switch (cmd_name[1])
			{
				// MA: Stop All Motors and Aux Ports
				case 'A':
					hostcmd_ma(); break;
				// MC: Start Coordinated Move
				case 'C':
					hostcmd_mc(); break;
				// MI: Start All Motors, Immediate Mode
				case 'I':
					hostcmd_mi(); break;
				// MM: Stop Single Motor
				case 'M':
					hostcmd_mm(); break;
				// MS: Start Single Motor
				case 'S':
					hostcmd_ms(); break;
				// MX: Start an XYZ Move
				case 'X':
					hostcmd_mx(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'O':
			switch (cmd_name[1])
			{
				// OB: Set Output Bit
				case 'B':
					hostcmd_ob(); break;
				// OP: Set Output Port
				case 'P':
					hostcmd_op(); break;
				// OR: Read Output Port
				case 'R':
					hostcmd_or(); break;
				// OT: Toggle Output Bit
				case 'T':
					hostcmd_ot(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'P':
			switch (cmd_name[1])
			{
				// PA: Read Actual Position
				case 'A':
					hostcmd_pa(); break;
				// PW: Read Destination Position
				case 'W':
					hostcmd_pw(); break;
				// PZ: Read XYZ Destination Position
				case 'Z':
					hostcmd_pz(); break;
				// PD: Set Destination Position, Absolute
				case 'D':
					hostcmd_pd(); break;
				// PR: Set Destination Position, Relative
				case 'R':
					hostcmd_pr(); break;
				// PX: Set XYZ Destination, Absolute
				case 'X':
					hostcmd_px(); break;
				// PY Set XYZ Destination, Relative
				case 'Y':
					hostcmd_py(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'R':
			switch (cmd_name[1])
			{
				// RL: Read Limit Switches
				case 'L':
					hostcmd_rl(); break;
				// RA: Read Proportional Gain
				case 'A':
					hostcmd_ra(); break;
				// RB: Read Differential Gain
				case 'B':
					hostcmd_rb(); break;
				// RC: Read Integral Gain
				case 'C':
					hostcmd_rc(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'S':
			switch (cmd_name[1])
			{
				// SA: Read Motor Status
				case 'A':
					hostcmd_sa(); break;
				// SC: Read System Configuration
				case 'C':
					hostcmd_sc(); break;
				// SD: Stop/Start Delay Timer
				case 'D':
					hostcmd_sd(); break;
				// SE: Read Host Error Stack
				case 'E':
					hostcmd_se(); break;
				// SM: Read Motor Mode
				case 'M':
					hostcmd_sm(); break;
				// SP: Read Teach Pendant Error Byte
				case 'P':
					hostcmd_sp(); break;
				// SR: Reset Motor Current Limit Circuitry
				case 'R':
					hostcmd_sr(); break;
				// SS: Read System Status
				case 'S':
					hostcmd_ss(); break;
				// ST: Execute Diagnostics
				case 'T':
					hostcmd_st(); break;
				// SU: Read Usage Time
				case 'U':
					hostcmd_su(); break;
				// SV: Read Version and I.D. Number
				case 'V':
					hostcmd_sv(); break;
				// SX: Execute Diagnostics and Return Results
				case 'X':
					hostcmd_sx(); break;
				// SZ: Read the Delay Timer Value
				case 'Z':
					hostcmd_sz(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'T':
			switch (cmd_name[1])
			{
				// TA: Abort/Terminate Teach Pendant Program
				case 'A':
					hostcmd_ta(); break;
				// TC: Clear Teach Pendant Display
				case 'C':
					hostcmd_tc(); break;
				// TD: Print to Teach Pendant Display
				case 'D':
					hostcmd_td(); break;
				// TE: Enable/Disable Teach Pendant to Move Motors
				case 'E':
					hostcmd_te(); break;
				// TH: Give Control to Host
				case 'H':
					hostcmd_th(); break;
				// TX: Give Control to Teach Pendant
				case 'X':
					hostcmd_tx(); break;
				// TK: Return to Host the Next Key Code
				case 'K':
					hostcmd_tk(); break;
				// TL: Return to Host the Last Key Code
				case 'L':
					hostcmd_tl(); break;
				// TR: Reset the Teach Pendant
				case 'R':
					hostcmd_tr(); break;
				// TS: Set Teach Pendant Display Cursor
				case 'S':
					hostcmd_ts(); break;
				// TT: Execute Teach Pendant Diagnostics and Return Results
				case 'T':
					hostcmd_tt(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'U':
			switch (cmd_name[1])
			{
				// UA: Read XYZ Rotation Angle
				case 'A':
					hostcmd_ua(); break;
				// UH: Read XYZ Home Position
				case 'H':
					hostcmd_uh(); break;
				// UO: Read XYZ Offset
				case 'O':
					hostcmd_uo(); break;
				// UT: Read Tool Length
				case 'T':
					hostcmd_ut(); break;
				// UY: Read Height of Elbow Rotation Axis
				case 'Y':
					hostcmd_uy(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'V':
			switch (cmd_name[1])
			{
				// VA: Read Motor Actual Velocity
				case 'A':
					hostcmd_va(); break;
				// VR: Read Motor Desired Velocity
				case 'R':
					hostcmd_vr(); break;
				// VX: Read System Velocity
				case 'X':
					hostcmd_vx(); break;
				// VG: Set System Velocity
				case 'G':
					hostcmd_vg(); break;
				// VS: Set Motor Velocity
				case 'S':
					hostcmd_vs(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'W':
			switch (cmd_name[1])
			{
				// WA: Abort all Waits
				case 'A':
					hostcmd_wa(); break;
				// WI: Wait on Input or Switch
				case 'I':
					hostcmd_wi(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		case 'X':
			switch (cmd_name[1])
			{
				// XR: Read Auxiliary Port Level and Direction
				case 'R':
					hostcmd_xr(); break;
				// XA: Set XYZ Rotation Angle
				case 'A':
					hostcmd_xa(); break;
				// XH: Set XYZ Home Position
				case 'H':
					hostcmd_xh(); break;
				// XO: Set XYZ Offset
				case 'O':
					hostcmd_xo(); break;
				// XS: Set Aux Port Level and Direction
				case 'S':
					hostcmd_xs(); break;
				// XT: Set Tool Length
				case 'T':
					hostcmd_xt(); break;
				// XY: Set Height of Elbow Rotation Axis
				case 'Y':
					hostcmd_xy(); break;
				default:
					// error: unknown command
					dbgmsg_uart2(ERROR);
					break;
			}
			break;
		default:
			// error: unknown command
			dbgmsg_uart2(ERR_UNKOWN_CMD);
			break;
	}
}

/******************************************************************************
 *                    HOST COMMAND FUNCTION IMPLEMENTATION                    *
 ******************************************************************************/

#include <stdio.h> // snprintf

/**
 * Read Motor Status.
 * 
 * Returns which motors are executing a trapezoidal move.
 * 
 * The trapezoidal move status of each motor is contained in a single byte within the controller.
 * Each bit within the status byte corresponds to one motor. Bit 0 corresponds to motor port A.
 * If the bit is set (1), the corresponding motor is still executing a trapezoidal move. If the bit
 * is cleared (0), the corresponding motor is either stationary or in a mode other than trapezoidal.
 */
inline void hostcmd_sa(void)
{
	char buf[64];
	update_motor_status();
	snprintf(buf, 64, "%u" CMDEND, controller.motor_status);
	hostcom_send(buf);
}

/**
 * Read System Configuration.
 * 
 * Returns the system configuration byte representing eight system mode selects.
 * 
 * The value returned is the decimal representation of the byte and ranges from 0 to 255. The value
 * returned must be decoded to determine the state of the various system conditions.
 * 
 * Bit 7:
 * 
 *     1 = system is in host mode
 *     0 = system is in pendant mode
 * 
 * Bit 6:
 * 
 *     1 = the pendant is enabled
 *     0 = the pendant is disabled
 * 
 * Bit 5:
 * 
 *     1 = generic controller mode
 *     0 = robot controller mode
 * 
 * Bit 4:
 * 
 *     1 = SCARA mode
 *     0 = XR-3 mode
 * 
 * Bit 3:
 * 
 *     1 = the gripper is disabled
 *     0 = the gripper is enabled
 * 
 * Bit 2:
 * 
 *     1 = XYZ mode
 *     0 = joint mode
 * 
 * Bit 1: always 0
 * 
 * Bit 0: allways 0
 */
inline void hostcmd_sc(void)
{
	char buf[64];
	snprintf(buf, 64, "%u" CMDEND, controller.system_config);
	hostcom_send(buf);
}

/**
 * Stop/Start Delay Timer.
 * 
 * Controls a general purpose timer in the controller.
 */
inline void hostcmd_sd(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			if (0 <= intparam1 && intparam1 <= 3000)
			{
				// set timer
			}
			else
			{
				// error: parameter out of range
				dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			// error: parameter must be an integer
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Host Error Stack.
 * 
 * Returns the last value pushed onto the error stack.
 * 
 * A first-in-last-out register, able to hold 24 pieces of information, is maintained in the controller
 * for holding error codes. If a system error should occur, the code representing the error is pushed
 * onto the stack.
 * 
 * Bit 6 of the system status byte reflects the status of the error stack. If the bit is set (1), an
 * error exists and the error stack should be read to determine the source of the error. The bit remains
 * set until the error stack is empty. SE returns zero if the error stack is empty.
 */
inline void hostcmd_se(void)
{
	char buf[64];
	unsigned char error_code = 0;
	snprintf(buf, 64, "%u" CMDEND, error_code);
	hostcom_send(buf);
}

/**
 * Read Motor Mode.
 * 
 * Returns the specified motor's mode.
 */
inline void hostcmd_sm(void)
{
	char buf[64];
	motor_mode_t motor_mode = 0;
	bool_t error = false;

	update_motor_mode();
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'A':
					motor_mode = controller.motor_mode.motor_a;
					break;
				case 'B':
					motor_mode = controller.motor_mode.motor_b;
					break;
				case 'C':
					motor_mode = controller.motor_mode.motor_c;
					break;
				case 'D':
					motor_mode = controller.motor_mode.motor_d;
					break;
				case 'E':
					motor_mode = controller.motor_mode.motor_e;
					break;
				case 'F':
					motor_mode = controller.motor_mode.motor_f;
					break;
				case 'G':
					motor_mode = controller.motor_mode.motor_g;
					break;
				case 'H':
					motor_mode = controller.motor_mode.motor_h;
					break;
				default:
					// error: out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					error = true;
			}
			
			if (!error)
			{
				snprintf(buf, 64, "%u" CMDEND, motor_mode);
				hostcom_send(buf);
			}
		}
		else
		{
			// error: wrong type
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: missing params
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Teach Pendant Error Byte.
 * 
 * Returns the code of the last error recognized by the teach pendant.
 * 
 * Bit 0 of the system status byte reflects the status of the pendant error byte. If the bit is set (1),
 * an error exists and the pendant error byte should be read to determine the source of the error. SP returns
 * zero if no pendant error exists.
 */
inline void hostcmd_sp(void)
{
	char buf[64];
	char pendant_error = 0;
	snprintf(buf, 64, "%u", pendant_error);
	hostcom_send(buf);
}

/**
 * Reset Motor Current Limit Circuitry.
 * 
 * Reset all motor amplifier current limit circuits.
 */
inline void hostcmd_sr(void)
{

}

/**
 * Read system status.
 * 
 * Returns the system status byte representing eight system conditions.
 * 
 * The value returned is the decimal representation of the byte and ranges from 0 to 255. The value returned
 * must be decoded to determine the state of the various system conditions.
 * 
 * Bit 7: 1 = At least one motor is performing a trapezoidal move.
 * Bit 6: 1 = A system error as occurred.
 * Bit 5: 1 = The general purpose delay timer is active.
 * Bit 4: 1 = At least one wait on input or wait on switch is still pending.
 * Bit 3: 1 = No teach pendant is connected.
 * Bit 2: 1 = The teach pendant ENTER key has been pressed.
 * Bit 1: 1 = The teach pendant ESCAPE key has been pressed.
 * Bit 0: 1 = A teach pendant error has occurred.
 */
inline void hostcmd_ss(void)
{
	char buf[64];
	update_system_status();
	snprintf(buf, 64, "%u" CMDEND, controller.system_status);
	hostcom_send(buf);
}

/**
 * Execute Diagnostics.
 * 
 * Execute RAM test and teach pendant diagnostics.
 * 
 * If the RAM test fails, an error code is pushed onto the error stack. If the teach pendant is not
 * connected or if the teach pendant returns an error, and error code will be pushed onto the error
 * stack.
 */
inline void hostcmd_st(void)
{

}

/**
 * Read Usage Time.
 * 
 * Returns the amount of time the controller has been on since leaving the factory.
 * 
 * The value returned is in the range of 0 to 2147483647 in units of minutes. The usage timer is
 * updated once per minute. Therefore, turning on the controller for less than one minute will
 * have no effect on the usage time stored.
 */
inline void hostcmd_su(void)
{
	char buf[64];
	snprintf(buf, 64, "%lu\n", controller.usage_time);
	hostcom_send(buf);
}

/**
 * Read Version and I.D. Number.
 * 
 * Returns the version of the controller and its (unique) serial or identification number.
 * 
 * The controller returns a string containing a copyright notice, the version of firmware
 * being used and the serial number.
 */
inline void hostcmd_sv(void)
{
	#define CONTROLLER_VERSION "Copyright (C) 2013 by Jonan Cruz-Martin V 0.1.0 SN XXXX.\n"
	hostcom_send(CONTROLLER_VERSION);
}

/**
 * Execute Diagnostics and Return Results.
 * 
 * Execute RAM and teach pendant diagnostics and return the results.
 * 
 * If the RAM test fails, an error code is pushed onto the error stack. If the teach pendant is not
 * connected or if the teach pendant returns an error, and error code will be pushed onto the error
 * stack.
 */
inline void hostcmd_sx(void)
{
	#define TEACH_PENDANT_ONLINE "Teach Pendant:  Online.\n"
	#define TEACH_PENDANT_OFFLINE "Teach Pendant:  Offline/Error.\n"
	#define RAM_TEST_PASSED "Ram Test:  Passed.  Last Addr= %xH.  Bytes OK = %u.\n"
	#define RAM_TEST_FAILED "Ram Test:  FAILED.  Last Addr= %xH.  Bytes OK = %u.\n"
	
	bool_t test_passed = false;
	unsigned int last_addr, bytes_ok;
	char buf[64];
	
	test_passed = test_teach_pendant();
	if (test_passed)
		snprintf(buf, 64, TEACH_PENDANT_ONLINE);
	else
		snprintf(buf, 64, TEACH_PENDANT_OFFLINE);
	hostcom_send(buf);
	
	test_passed = test_ram(&last_addr, &bytes_ok);
	if (test_passed)
		snprintf(buf, 64, RAM_TEST_PASSED, last_addr, bytes_ok);
	else
		snprintf(buf, 64, RAM_TEST_FAILED, last_addr, bytes_ok);
	hostcom_send(buf);
}

/**
 * Read the Delay Timer Value.
 * 
 * Returns the current value in the general purpose timer.
 * 
 * The value returned can range from 0 to 3000 in units of 1/10 second.
 */
inline void hostcmd_sz(void)
{
	char buf[64];
	snprintf(buf, 64, "%d\n", controller.delay_timer);
	hostcom_send(buf);
}

/**
 * Set Coordinate Position.
 * 
 * Converts encoder position to xyz position and vice versa.
 * 
 * A value of d = 0 causes the absolute destination registers to be set to the values
 * corresponding to the current xyz position. Normally, this is already the case. A value
 * of d = 1 causes the xyz destination registers to be set to the values corresponding to
 * the current encoder position.
 */
inline void hostcmd_cc(void)
{
	if (any_motor_executing_trapezoidal_move(MOTOR_ALL))
	{
		// error
		dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
	}
	else
	{
		if (hard_home_executed())
		{
			if (controller_in_robot_mode())
			{
				if (param1.present)
				{
					if (param1.type == TOKEN_INT)
					{
						int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
						switch (intparam1)
						{
							case 0:
								break;
							case 1:
								break;
							default:
								// error
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error
					dbgmsg_uart2(ERR_MISSING_PARAMS);
				}
			}
			else
			{
				// error
				dbgmsg_uart2(ERR_NOT_ROBOT_MODE);
			}
		}
		else
		{
			// error
			dbgmsg_uart2(ERR_NO_HARD_HOME);
		}
	}
}

/**
 * Enable/Disable Gripper Mode.
 * 
 * Enable or disable the XR-3 or SCARA gripper.
 * 
 * A value of d = 0 disables the gripper and d = 1 enables the gripper.
 */
inline void hostcmd_cg(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (controller_in_robot_mode())
		{
			if (param1.present)
			{
				if (param1.type == TOKEN_INT)
				{
					int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
					switch (intparam1)
					{
						case 0:
							disable_gripper();
							break;
						case 1:
							enable_gripper();
							break;
						default:
							// error
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
		else
		{
			// error: controller must be in robot mode
			dbgmsg_uart2(ERR_NOT_ROBOT_MODE);
		}
	}
}

/**
 * Set Motor Mode.
 * 
 * Set the specified motor's mode to idle, trapezoidal, velocity or open loop.
 * 
 * The following table shows the valid modes and the corresponding values for d:
 * 
 * 0) Idle mode.
 * 1) Trapezoidal mode.
 * 2) Velocity mode.
 * 3) Open Loop mode.
 */
inline void hostcmd_cm(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (hard_home_in_progress())
		{
			// error: hard home in progress
			dbgmsg_uart2(ERR_EXECUTING_HARD_HOME);
		}
		else
		{
			if (param1.present)
			{
				if (param1.type == TOKEN_LETTER)
				{
					if (param2.present)
					{
						if (param2.type == TOKEN_INT)
						{
							int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
							if (0 <= intparam2 && intparam2 <= 3)
							{
								switch (param1.value.letter)
								{
									case 'A':
										set_motor_mode(MOTOR_A, intparam2);
										break;
									case 'B':
										set_motor_mode(MOTOR_B, intparam2);
										break;
									case 'C':
										set_motor_mode(MOTOR_C, intparam2);
										break;
									case 'D':
										set_motor_mode(MOTOR_D, intparam2);
										break;
									case 'E':
										set_motor_mode(MOTOR_E, intparam2);
										break;
									case 'F':
										set_motor_mode(MOTOR_F, intparam2);
										break;
									case 'G':
										set_motor_mode(MOTOR_G, intparam2);
										break;
									case 'H':
										set_motor_mode(MOTOR_H, intparam2);
										break;
									default:
										// error : parameter 1 out of range
										dbgmsg_uart2(ERR_OUT_OF_RANGE);
								}
							}
							else
							{
								// error: parameter 2 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 must be an integer number
							dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
						}
					}
					else
					{
						// error: parameter 2 must be specified
						dbgmsg_uart2(ERR_MISSING_PARAMS);
					}
				}
				else
				{
					// error: parameter 1 must be an integer number
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 1 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
	}
}

/**
 * Set Robot Type.
 * 
 * Set the controller to control an XR-3, a SCARA or no robot.
 * 
 * The following table shows the valid robots and the corresponding values for d:
 * 
 * 0) XR-3 controller.
 * 1) SCARA controller.
 * 2) GENERIC controller.
 */
inline void hostcmd_cr(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				if (any_motor_executing_trapezoidal_move(MOTOR_ALL))
				{
					// error: some motor is still executing a trapezoidal move
					dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
				}
				else
				{
					int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
					switch (intparam1)
					{
						case 0:
							set_controller_xr3_mode();
							break;
						case 1:
							set_controller_scara_mode();
							break;
						case 2:
							set_controller_generic_mode();
							break;
						default:
							// error: parameter 1 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Read System Acceleration.
 * 
 * Returns the system acceleration. The value returned is in the range of 0 to 100 and represents
 * the percentage of maximum system acceleration.
 */
inline void hostcmd_ar(void)
{
	char buf[64];
	update_system_acceleration();
	snprintf(buf, 64, "%u" CMDEND, controller.system_acceleration);
	hostcom_send(buf);
}

/**
 * Read Motor PWM Level and Direction.
 * 
 * Returns the specified motor's PWM level and its direction. The value returned ranges from -100
 * to 100 whose absolute magnitude represents the percentage of maximum motor power and whose sign
 * represents the direction the motor is turning in.
 */
inline void hostcmd_dr(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if ('A' <= param1.value.letter && param1.value.letter <= 'F')
			{
				const int size = 64;
				char buf[size];
				int recvd;

				buf[0] = 'C';
				buf[1] = param1.value.letter;
				buf[2] = *CMDEND;
				buf[3] = '\0';
				mcuicom_send(buf);
				recvd = mctlcom_get_response(buf, size);
				buf[recvd] = '\0';
				// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
				// Another possible solution may be to enable 'hostcom_send' to
				// accept a 'size' parameter that tells the size of the buffer.
				hostcom_send(buf);
			}
			else
			{
				// error: param 1 out of range
				dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Gripper Status.
 * 
 * Returns the status (open or closed) of the gripper. If the returned value is 1, the gripper is closed.
 * If the returned value is 0, the gripper is open. If the gripper is disabled, the returned value will be 0.
 */
inline void hostcmd_gs(void)
{
	const int size = 64;
	char buf[size];
	char gripper_status = 0;

	if (gripper_is_enabled())
		gripper_status = controller.gripper_status;
	snprintf(buf, size, "%u" CMDEND, gripper_status);
	hostcom_send(buf);
}

/**
 * Read Soft Home Position.
 * 
 * Returns the specified motor's soft home position.
 * 
 * The value returned ranges from -32767 to 32767 in encoder counts and represents the position the specified
 * motor would move to if an HG (go to soft home) were issued.
 * 
 * The soft home position is defined as 0 after power up or after executing a hard home. It is also defined as
 * the current motor position when an HS (set soft home) is issued.
 */
inline void hostcmd_hr(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			int       soft_home_pos;
			bool_t    error = false;
			
			switch (param1.value.letter)
			{
				case 'A':
					soft_home_pos = controller.soft_home_position.motor_a;
					break;
				case 'B':
					soft_home_pos = controller.soft_home_position.motor_b;
					break;
				case 'C':
					soft_home_pos = controller.soft_home_position.motor_c;
					break;
				case 'D':
					soft_home_pos = controller.soft_home_position.motor_d;
					break;
				case 'E':
					soft_home_pos = controller.soft_home_position.motor_e;
					break;
				case 'F':
					soft_home_pos = controller.soft_home_position.motor_f;
					break;
				case 'G':
					soft_home_pos = controller.soft_home_position.motor_g;
					break;
				case 'H':
					soft_home_pos = controller.soft_home_position.motor_h;
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					error = true;
			}
			
			if (!error)
			{
				char buf[64];
				snprintf(buf, 64, "%d\n", soft_home_pos);
				hostcom_send(buf);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Actual Position.
 * 
 * Returns the current or actual position of the specified motor. The value returned ranges from -32767
 * to 32767 in encoder counts.
 */
inline void hostcmd_pa(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if ('A' <= param1.value.letter && param1.value.letter <= 'F')
			{
				int size = 64;
				char buf[size];
				
				// Send MCUICOM command RA, RB, ..., RF depending on motor letter (param 1)
				buf[0] = 'R';
				buf[1] = param1.value.letter;
				buf[2] = *CMDEND;
				buf[3] = '\0';
				mcuicom_send(buf);
				// Get response (motor steps) and re-send it to the host PC
				size = mctlcom_get_response(buf, size);
				buf[size] = '\0'; // no need to check size, this is always < 64
				// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
				// Another possible solution may be to enable 'hostcom_send' to
				// accept a 'size' parameter that tells the size of the buffer.
				hostcom_send(buf);
			}
			else
			{
				// error
				dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Destination Position.
 * 
 * Returns the desired position of the specified motor. The value returned ranges from -32767 to
 * 32767 in encoder counts and represents the position the specified motor would move  to when
 * an MC (move coordinated), MI (move independent) or an MS (move single motor) command is issued.
 */
inline void hostcmd_pw(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			const int size = 64;
			char buf[size];
			int recvd;

			buf[0] = 'F';
			buf[1] = param1.value.letter;
			buf[2] = *CMDEND;
			buf[3] = '\0';
			mcuicom_send(buf);

			recvd = mctlcom_get_response(buf, size);
			buf[recvd] = '\0';
			// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
			// Another possible solution may be to enable 'hostcom_send' to
			// accept a 'size' parameter that tells the size of the buffer.
			hostcom_send(buf);
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read XYZ Destination Position.
 * 
 * Returns the desired xyz position of the specified axis.
 * 
 * The value returned represents the position in millimeters or degrees the robot would move to
 * when an MX (move xyz) command is issued. The X, Y and Z axes are in units of millimeters and
 * the A and T axes are in units of degrees. The value returned is fixed at two decimal places
 * (two digits after the decimal point).
 */
inline void hostcmd_pz(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			float     dest_val;
			bool_t    error = false;
			
			switch (param1.value.letter)
			{
				case 'X':
					dest_val = controller.xyz_destination.x;
					break;
				case 'Y':
					dest_val = controller.xyz_destination.y;
					break;
				case 'Z':
					dest_val = controller.xyz_destination.z;
					break;
				case 'A':
					dest_val = controller.xyz_destination.a;
					break;
				case 'T':
					dest_val = controller.xyz_destination.t;
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					error = true;
			}
			
			if (!error)
			{
				char buf[64];
				snprintf(buf, 64, "%.2f\n", (double) dest_val);
				hostcom_send(buf);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Limit Switches.
 * 
 * Returns the limit switch byte representing the state of each of the eight limit switches.
 *
 * The value returned is the decimal representation of the byte and ranges from 0 to 255. The
 * value returned must be decoded to determine the state of the various conditions.
 *
 * Bit 7:
 * 
 *     1 = Limit switch of motor H is open or inactive.
 *     0 = Limit switch of motor H is closed or active.
 * 
 * Bit 6:
 * 
 *     1 = Limit switch of motor G is open or inactive.
 *     0 = Limit switch of motor G is closed or active.
 * 
 * Bit 5:
 * 
 *     1 = Limit switch of motor F is open or inactive.
 *     0 = Limit switch of motor F is closed or active.
 * 
 * Bit 4:
 * 
 *     1 = Limit switch of motor E is open or inactive.
 *     0 = Limit switch of motor E is closed or active.
 * 
 * Bit 3:
 * 
 *     1 = Limit switch of motor D is open or inactive.
 *     0 = Limit switch of motor D is closed or active.
 * 
 * Bit 2:
 * 
 *     1 = Limit switch of motor C is open or inactive.
 *     0 = Limit switch of motor C is closed or active.
 * 
 * Bit 1:
 * 
 *     1 = Limit switch of motor B is open or inactive.
 *     0 = Limit switch of motor B is closed or active.
 * 
 * Bit 0:
 * 
 *     1 = Limit switch of motor A is open or inactive.
 *     0 = Limit switch of motor A is closed or active.
 */
inline void hostcmd_rl(void)
{
	const int size = 64;
	char buf[size];
	update_limit_switches();
	snprintf(buf, size, "%u" CMDEND, controller.limit_switches);
	hostcom_send(buf);
}

/**
 * Read XYZ Rotation Angle.
 * 
 * Returns the angle of rotation of the user's coordinate system with respect to the
 * robot coordinate system.
 * 
 * The value returned is a floating point number in units of degrees.
 */
inline void hostcmd_ua(void)
{
}

/**
 * Read XYZ Home Position.
 * 
 * Returns the linear distance between the robot coordinate system origin and the
 * center of the tool tip (gripper).
 * 
 * The value returned is a floating point number in units of millimeters for the X,
 * Y and Z axes and units of degrees for the A and T axes.
 * 
 * The A axis does not exist on the SCARA robot.
 */
inline void hostcmd_uh(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'X':
					break;
				case 'Y':
					break;
				case 'Z':
					break;
				case 'A':
					break;
				case 'T':
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read XYZ Offset.
 * 
 * Returns the linear or angular displacement between the user coordinate system
 * and the robot coordinate system.
 * 
 * The value returned is a floating point number in units of millimeters for the X,
 * Y and Z axes and units of degrees for the A and T axes.
 * 
 * The A axis does not exist on the SCARA robot.
 */
inline void hostcmd_uo(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'X':
					break;
				case 'Y':
					break;
				case 'Z':
					break;
				case 'A':
					break;
				case 'T':
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Tool Length.
 * 
 * Returns the distance from the hand flex axis to the toolt tip (gripper end).
 * 
 * The value returned is a floating point number in units of millimeters.
 */
inline void hostcmd_ut(void)
{
}

/**
 * Read Height of Elbow Rotation Axis.
 * 
 * Returns the height of the elbow rotation axis from the reference surface.
 * 
 * The value returned is a floating point number in units of millimeters.
 * 
 * This parameter has no meaning if the current robot type is SCARA.
 */
inline void hostcmd_uy(void)
{
}

/**
 * Read Motor Actual Velocity.
 * 
 * Returns the actual motor velocity of the specified motor.
 * 
 * The value returned ranges from -100 to 100 whose absolute magnitude represents the percentage
 * of maximum motor velocity and whose sign represents the direction the motor is turning in.
 */
inline void hostcmd_va(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			char      velocity;
			bool_t    error = false;
			
			switch (param1.value.letter)
			{
				case 'A':
					velocity = controller.motor_actual_velocity.motor_a;
					break;
				case 'B':
					velocity = controller.motor_actual_velocity.motor_b;
					break;
				case 'C':
					velocity = controller.motor_actual_velocity.motor_c;
					break;
				case 'D':
					velocity = controller.motor_actual_velocity.motor_d;
					break;
				case 'E':
					velocity = controller.motor_actual_velocity.motor_e;
					break;
				case 'F':
					velocity = controller.motor_actual_velocity.motor_f;
					break;
				case 'G':
					velocity = controller.motor_actual_velocity.motor_g;
					break;
				case 'H':
					velocity = controller.motor_actual_velocity.motor_h;
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					error = true;
			}
			
			if (!error)
			{
				char buf[64];
				snprintf(buf, 64, "%u" CMDEND, velocity);
				hostcom_send(buf);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Motor Desired Velocity.
 * 
 * Returns the desired velocity of the specified motor.
 * 
 * The value returned ranges from -100 to 100 whose absolute magnitude represents the percentage
 * of maximum motor velocity and whose sign represents the direction the motor is turning in. For
 * trapezoidal mode motors, the direction has no meaning, being a function of the desired position.
 * 
 * Whenever the gripper is enabled or commanded to open or close, a factory set velocity will be used
 * for the gripper. The motor velocity read will have no meaning.
 * 
 * While under teach pendant mode, the motor velocity returned may not be what was programmed depending
 * on the mode the pendant is in or the function being executed. Once the system returns to play mode
 * and no function is being executed, the motor velocity will return to its original value.
 */
inline void hostcmd_vr(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if ('A' <= param1.value.letter && param1.value.letter <= 'F')
			{
				const int size = 64;
				char buf[size];
				int recvd;

				buf[0] = 'Y';
				buf[1] = param1.value.letter;
				buf[2] = *CMDEND;
				buf[3] = '\0';
				mcuicom_send(buf);
				recvd = mctlcom_get_response(buf, size);
				buf[recvd] = '\0';
				// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
				// Another possible solution may be to enable 'hostcom_send' to
				// accept a 'size' parameter that tells the size of the buffer.
				hostcom_send(buf);
			}
			else
			{
				// error: param 1 out of range
				dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			// error: param 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: param 1 must be provided
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read System Velocity.
 * 
 * Returns the system velocity. The value returned ranges from 0 to 100 and represents the percentage
 * of maximum system velocity.
 */
inline void hostcmd_vx(void)
{
	const int size = 64;
	char buf[size];
	int recvd;

	mcuicom_send("RV" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
	// Another possible solution may be to enable 'hostcom_send' to
	// accept a 'size' parameter that tells the size of the buffer.
	hostcom_send(buf);
}

/**
 * Read Auxiliary Port Level and Direction.
 * 
 * Returns the specified auxiliary port's PWM level and its direction.
 * 
 * The value returned ranges from -100 to 100 whose absolute magnitude represents the percentage
 * of maximum motor velocity and whose sign represents the direction the motor is turning in.
 */
inline void hostcmd_xr(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			switch (intparam1)
			{
				case 0:
					break;
				case 1:
					break;
				default:
					// error
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			// error
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Clear Actual Position.
 * 
 * Set the actual position of the specified motor to 0.
 * 
 * If the specified motor is in trapezoidal mode, it must not be currently executing a trapezoidal
 * move. This can be checked by issuing the SA command.
 * 
 * This command is normally used during startup to initialize a motor when a home on switch is not
 * available. Especially useful under generic controller mode when limit switches are not being used.
 */
inline void hostcmd_ac(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if ('A' <= param1.value.letter && param1.value.letter <= 'F')
				{
					unsigned char motor = 1 << (param1.value.letter - 'A');
					// If motor is in trapezoidal mode and executing a trapezoidal move,
					// the actual position of the motor cannot be cleared.
					if (any_motor_executing_trapezoidal_move(motor))
					{
					// error: motor cannot be executing trapezoidal move
						dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
					}
					// If the motor is not in trapezoidal mode or is not executing a trapezoidal move,
					// the actual position can be cleared safely.
					else
					{
						const int size = 64;
						char buf[size];

						// Disable PID control on the given motor, to prevent
						// that motor from moving to correct the position.
						buf[0] = 'D';
						buf[1] = param1.value.letter;
						buf[2] = *CMDEND;
						buf[3] = '\0';
						mcuicom_send(buf);

						// Clear the actual position register of the MCMCU for
						// the given motor.
						buf[0] = 'K';
						mcuicom_send(buf);

						// Set the desired position register of the MCMCU to
						// position 0.
						// Otherwise, if this register were non-zero, the motor
						// would move that amount of steps relative to the new
						// zero position, which is undesired.
						buf[0] = 'G';
						buf[2] = ',';
						buf[3] = '0';
						buf[4] = *CMDEND;
						buf[5] = '\0';
						mcuicom_send(buf);

						// Re-enable PID control on the given motor
						buf[0] = 'E';
						buf[2] = *CMDEND;
						buf[3] = '\0';
						mcuicom_send(buf);
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set System Acceleration.
 * 
 * Set system acceleration as a percentage of system maximum acceleration.
 * 
 * System acceleration is a global parameter that affects all motors. If system acceleration is 0,
 * motors in trapezopidal mode will not be able to move and motors in velocity mode will be stopped.
 * 
 * Motors in trapezoidal mode must not be currently executing a trapezoidal move. This can be checked
 * by issuing the SS command.
 * 
 * Motors in velocity mode will immediately begin using the new acceleration.
 */
inline void hostcmd_as(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (0 <= intparam1 && intparam1 <= 100)
				{
					if (any_motor_executing_trapezoidal_move(MOTOR_ALL))
					{
						// error: motors in trapezoidal mode must not be executing a trapezoidal move
						dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
					}
					else
					{
						const int size = 64;
						char buf[size];
						snprintf(buf, size, "AS,%d" CMDEND, intparam1);
						mcuicom_send(buf);
						controller.system_acceleration = intparam1;
					}
				}
				else
				{
					// error: value of paramter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set PWM Level and Direction.
 * 
 * Sets an open loop mode motor's pwm level and direction.
 * 
 * The specified motor must be in open loop mode. This can be checked by using an SM command.
 * PWM level is a percentage of the absolute value of maximum motor power. A minus sign indicates
 * negative voltage or direction.
 * 
 * The command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ds(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param2.present)
			{
				if (param1.type == TOKEN_LETTER)
				{
					if (param2.type == TOKEN_INT)
					{
						int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
						if (-100 <= intparam2 && intparam2 <= 100)
						{
							// TODO: check if the motor is in open-loop mode?

							const int size = 64;
							char buf[size];

							snprintf(buf, size, "P%c,%d" CMDEND, param1.value.letter, intparam2);
							mcuicom_send(buf);
						}
						else
						{
							// error: parameter 2 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 must be an integer number
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error: parameter 1 must be a letter
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 2 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
		else
		{
			// error: paramter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Close Gripper.
 * 
 * The system must be in XR-3 or SCARA mode with the gripper enabled. The gripper
 * must not be currently executing a move. This can be checked by issuing the SA
 * command.
 * 
 * When the grippper is commanded to close, the motor moves to a position beyond
 * which the gripper can travel. This means that, when the gripper closes on an
 * object, full motor power is applied. Stall detection on closing is disabled.
 * Stall will be detected if the gripper is commanded to open and cannot. Velocity
 * is at a factory set value and cannot be modified.
 * 
 * The gripper is controlled by motor port A. Consequently, motor port A's absolute
 * destination and velocity registers are modified.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_gc(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (controller_mode() == XR3 || controller_mode() == SCARA)
		{
			if (gripper_is_enabled())
			{
				// TODO: close gripper
				
				// Switch gripper status to closed
				controller.gripper_status = GRIPPER_CLOSED;
			}
			else
			{
				// error: the gripper must be enabled
				dbgmsg_uart2(ERR_GRIPPER_NOT_ENABLED);
			}
		}
		else
		{
			// error: controller must be in robot mode (either XR-3 or SCARA)
			dbgmsg_uart2(ERR_NOT_ROBOT_MODE);
		}
	}
}

/**
 * Open Gripper.
 * 
 * The system must be in XR-3 or SCARA mode with the gripper enabled. The gripper
 * must not be currently executing a move. This can be checked by issuing the SA
 * command.
 * 
 * When the grippper is commanded to close, the motor moves to a position beyond
 * which the gripper can travel. This means that, when the gripper closes on an
 * object, full motor power is applied. Stall detection on closing is disabled.
 * Stall will be detected if the gripper is commanded to open and cannot. Velocity
 * is at a factory set value and cannot be modified.
 * 
 * The gripper is controlled by motor port A. Consequently, motor port A's absolute
 * destination and velocity registers are modified.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_go(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (controller_mode() == XR3 || controller_mode() == SCARA)
		{
			if (gripper_is_enabled())
			{
				// TODO: open gripper
				
				// Switch gripper status to open
				controller.gripper_status = GRIPPER_OPEN;
			}
			else
			{
				// error: the gripper must be enabled
				dbgmsg_uart2(ERR_GRIPPER_NOT_ENABLED);
			}
		}
		else
		{
			// error: controller must be in robot mode (either XR-3 or SCARA)
			dbgmsg_uart2(ERR_NOT_ROBOT_MODE);
		}
	}
}

/**
 * Go To the Hard Home Position.
 * 
 * Moves all motors that are in trapezoidal mode to their 0 encoder position.
 * 
 * A hard home must have been previously executed.
 * 
 * If the controller is under XR-3 mode, motors B through F will move in a coordinated
 * fashion. The busy status of motors B through F should be checked with the SA command.
 * 
 * If the controller is under SCARA mode, motors B through E will move in a coordinated
 * fashion. The busy status of motors B through E should be checked with the SA command.
 * 
 * If the controller is under GENERIC mode, all motors under trapezoidal mode move
 * according to their set motor velocities. The busy status of all motors in trapezoidal
 * mode should be checked with the SA or SS command.
 * 
 * Motor destination registers of affected motors become 0.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ha(void)
{
	// TODO: test command
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (hard_home_executed())
		{
			// If the controller is in generic mode, go to hard home position on
			// motor A, too, unless the gripper is enabled on motor A.
			if (controller_mode() == GENERIC && !gripper_is_enabled())
				mcuicom_send("AA,0" CMDEND);
			// TODO: According to the manual, the gripper motor shouldn't be
			// affected if it's enabled. By default, if the controller is in
			// robot mode, the gripper motor (motor A) is never affected. Thus,
			// it is assumed that the manual refers to the case when the
			// controller is in generic mode. Being so, it is also assumed that
			// the gripper motor, if enabled, will always be motor A. This
			// should be checked in the manual, to make sure that the assumption
			// is correct.

			mcuicom_send("AB,0" CMDEND);
			mcuicom_send("AC,0" CMDEND);
			mcuicom_send("AD,0" CMDEND);
			mcuicom_send("AE,0" CMDEND);

			if (controller_mode() != SCARA)
				mcuicom_send("AF,0" CMDEND);

			if (controller_mode() == GENERIC)
			{
				// According to the manual, if the controller is under generic
				// mode, all motors under trapezoidal mode should move according
				// to their set motor velocities. Since a coordinated movement
				// would not satisfy this requirement, it is assumed that the
				// manual refers to an independent movement, even though it is
				// not mentioned as such.
				mcuicom_send("MI" CMDEND);
				// FIXME: From the manual, it is understood that only the motors
				// in trapezoidal mode should move to the hard home position, but
				// not any other motor if it is in any other motor mode.
			}
			else
			{
				// FIXME: According to the manual, the movement should be
				// coordinated, but the coordinated movement isn't implemented
				// yet (see 'motorctl' program).
				mcuicom_send("MI" CMDEND);
			}
		}
		else
		{
			// error: a hard home must have been executed
			dbgmsg_uart2(ERR_NO_HARD_HOME);
		}
	}
}

/**
 * Go To the Soft Home Position.
 * 
 * Moves all motors that are in trapezoidal mode to their soft home position.
 * 
 * If the controller is under XR-3 mode, motors B through F will move in a coordinated
 * fashion. The busy status of motors B through F should be checked with the SA command.
 * 
 * If the controller is under SCARA mode, motors B through E will move in a coordinated
 * fashion. The busy status of motors B through E should be checked with the SA command.
 * 
 * If the controller is under GENERIC mode, all motors under trapezoidal mode move
 * according to their set motor velocities. The busy status of all motors in trapezoidal
 * mode should be checked with the SA or SS command.
 * 
 * The gripper motor is not affected if enabled.
 * 
 * Motor destination registers of affected motors are set to their soft home position.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_hg(void)
{
	// TODO: test command
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (hard_home_executed())
		{
			const int size = 64;
			char buf[size];

			// If the controller is in generic mode, go to hard home position on
			// motor A, too, unless the gripper is enabled on motor A.
			if (controller_mode() == GENERIC && !gripper_is_enabled())
			{
				snprintf(buf, size, "AA,%d" CMDEND, controller.soft_home_position.motor_a);
				mcuicom_send(buf);
			}
			// TODO: According to the manual, the gripper motor shouldn't be
			// affected if it's enabled. By default, if the controller is in
			// robot mode, the gripper motor (motor A) is never affected. Thus,
			// it is assumed that the manual refers to the case when the
			// controller is in generic mode. Being so, it is also assumed that
			// the gripper motor, if enabled, will always be motor A. This
			// should be checked in the manual, to make sure that the assumption
			// is correct.


			snprintf(buf, size, "AB,%d" CMDEND, controller.soft_home_position.motor_b);
			mcuicom_send(buf);

			snprintf(buf, size, "AC,%d" CMDEND, controller.soft_home_position.motor_c);
			mcuicom_send(buf);

			snprintf(buf, size, "AD,%d" CMDEND, controller.soft_home_position.motor_d);
			mcuicom_send(buf);

			snprintf(buf, size, "AE,%d" CMDEND, controller.soft_home_position.motor_e);
			mcuicom_send(buf);

			if (controller_mode() != SCARA)
			{
				snprintf(buf, size, "AF,%d" CMDEND, controller.soft_home_position.motor_f);
				mcuicom_send(buf);
			}

			if (controller_mode() == GENERIC)
			{
				// According to the manual, if the controller is under generic
				// mode, all motors under trapezoidal mode should move according
				// to their set motor velocities. Since a coordinated movement
				// would not satisfy this requirement, it is assumed that the
				// manual refers to an independent movement, even though it is
				// not mentioned as such.
				mcuicom_send("MI" CMDEND);
				// FIXME: From the manual, it is understood that only the motors
				// in trapezoidal mode should move to the soft home position, but
				// not any other motor if it is in any other motor mode.
			}
			else
			{
				// FIXME: According to the manual, the movement should be
				// coordinated, but the coordinated movement isn't implemented
				// yet (see 'motorctl' program).
				mcuicom_send("MI" CMDEND);
			}
		}
		else
		{
			// error: a hard home must have been executed
			dbgmsg_uart2(ERR_NO_HARD_HOME);
		}
	}
}

/**
 * Execute a Hard Home.
 * 
 * Moves motors that are in trapezoidal mode to their hard limit switch positions.
 * 
 * If any motor is in trapezoidal mode, it must not be currently executing a trapezoidal
 * move. This can be checked by issuing an SS command.
 * 
 * If the controller is under XR-3 mode, motors B through F will move to their hard limit
 * switch position and their actual destination positions will be set to 0. The gripper,
 * if enabled, will close and, after all motors have found their 0 position, will open.
 * 
 * If the controller is under SCARA mode, motors B through E will move to their hard limit
 * switch position and their actual destination positions will be set to 0. The gripper,
 * if enabled, will close and, after all motors have found their 0 position, will open.
 * 
 * If an error occurs, the hard home execution will be terminated and the gripper will open.
 * 
 * If the controller is under GENERIC mode, all motors under trapezoidal mode will have
 * their actual and destination positions set to 0 but no movement will take place.
 * 
 * The hard limit switch position is found by taking the motor position at both ends of
 * switch closure and calculating the center of the switch. The switch is always approached
 * from the same direction for purposes of counting the encoder states as the switch is
 * traveled across.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_hh(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (any_motor_executing_trapezoidal_move(MOTOR_ALL))
		{
			// error: no motor can be executing a trapezoidal move
			dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
		}
		else
		{
			hardhome();
		}
	}
}

/**
 * Hard Home on Limit Switch.
 * 
 * Moves a motor that is in trapezoidal mode to its hard limit switch position.
 * 
 * The optional parameter d specifies an initial direction to look for a switch closure.
 * If, when d is specified, the switch is not found or the motor stalls, the routine fails.
 * When d is not specified, the motor will change direction after a first failure and try
 * again. The routine fails if the switch is again not found or a stall occurs a second time.
 * The parameter d is provided for those systems where the limit switch is at an end of travel
 * and HL without d would cause the motor to initially travel in the wrong direction.
 * 
 * If the specified motor is in trapezoidal mode, it must not be currently executing a trapezoidal
 * move. This can be checked by issuing the SA command.
 * 
 * The specified motor's actual and destination registers are set to 0.
 * 
 * This command is normally used to hard home those motors with limit switches that the HH command
 * does not handle. For example, motors G and H when the controller is under XR-3 mode or motors A
 * through H when the controller is under generic mode. This command is invalid for motor A if it is
 * enabled as the gripper port.
 * 
 * The hard limit switch position is found by taking the motor position at both ends of
 * switch closure and calculating the center of the switch. The switch is always approached
 * from the same direction for purposes of counting the encoder states as the switch is
 * traveled across.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_hl(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				int intparam2 = -1;
				bool_t param2_error = false;
				if (param2.present)
				{
					if (param2.type == TOKEN_INT)
					{
						intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
						if (intparam2 != 0 && intparam2 != 1)
							param2_error = true;
					}
					else
					{
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				
				if (param2_error)
				{
					// error: parameter 2 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
				else
				{
					switch (param1.value.letter)
					{
						case 'A':
							break;
						case 'B':
							break;
						case 'C':
							break;
						case 'D':
							break;
						case 'E':
							break;
						case 'F':
							break;
						case 'G':
							break;
						case 'H':
							break;
						default:
							// error: parameter 1 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
			}
			else
			{
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Set Soft Home.
 * 
 * Store the current motor position of all motors into their respective soft home
 * position register.
 * 
 * After power up, all joint coordinate soft home registers are set to 0. XYZ
 * coordinate soft home registers are unknown.
 * 
 * After a successful hard home, all affected motors have their joint coordinate
 * soft home registers set to 0 and the XYZ coordinate soft home registers are set
 * according to the robot selected.
 * 
 * Although valid at all times, motors in trapezoidal mode should not be currently
 * executing a trapezoidal move. This can be checked by issuing the SA command.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_hs(void)
{
	// TODO: test command
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		const int size = 64;
		char buf[size];
		int recvd;

		mcuicom_send("RA" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_a = atoi(buf);

		mcuicom_send("RB" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_b = atoi(buf);

		mcuicom_send("RC" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_c = atoi(buf);

		mcuicom_send("RD" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_d = atoi(buf);

		mcuicom_send("RE" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_e = atoi(buf);

		mcuicom_send("RF" CMDEND);
		recvd = mctlcom_get_response(buf, size);
		buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
		controller.soft_home_position.motor_f = atoi(buf);

		// TODO: set xyz soft home position
		// FIXME: motors in trapezoidal move should not be executing a trapezoidal move
	}
}

/**
 * Stop All Motors and Aux Ports.
 * 
 * Stops all motors regardless of motor or controller mode. Turns off all
 * auxiliary ports.
 * 
 * Auxiliary port PWM level registers are set to 0. No other registers are
 * affected.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ma(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		// proceed
	}
}

/**
 * Start Coordinated Move.
 * 
 * Valid motors in trapezoidal mode start and end movement at the same time.
 * 
 * Only motors under trapezoidal mode with a destination position different from
 * their current position are affected. Affected motors must have a non-zero motor
 * velocity. Motors whose relative destination registers are non-zero will make a
 * relative movement while all other motors will make an absolute movement. The
 * relative destination register becomes zero when the move command is issued.
 * 
 * If the controller is under XR-3 mode, only motors B-F are affected. Motors B
 * through F must not be executing a trapezoidal move and can be checked using the
 * SA command.
 * 
 * If the controller is under SCARA mode, only motors B-E are affected. Motors B
 * through E must not be executing a trapezoidal move and can be checked using the
 * SA command.
 * 
 * If the controller is under XR-3 mode, all motors are affected. No motor may be
 * executing a trapezoidal move. This can be checked by using either the SA or SS
 * command.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_mc(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		mcuicom_send("MC" CMDEND);
	}
}

/**
 * Start All Motors, Immediate Mode.
 * 
 * Motors in trapezoidal mode move to their destination positions under their
 * respective motor velocities.
 * 
 * Only motors under trapezoidal mode with a destination position different from
 * their current position are affected. Affected motors must have a non-zero motor
 * velocity. Motors whose relative destination registers are non-zero will make a
 * relative movement while all other motors will make an absolute movement. The
 * relative destination register becomes zero when the move command is issued.
 * 
 * Motors begin movement at the same time but may or may not stop at the same time,
 * depending on the distance each motor must move and the motor's set velocity.
 * 
 * If the controller is under XR-3 mode, only motors B-F are affected. Motors B
 * through F must not be executing a trapezoidal move and can be checked using the
 * SA command.
 * 
 * If the controller is under SCARA mode, only motors B-E are affected. Motors B
 * through E must not be executing a trapezoidal move and can be checked using the
 * SA command.
 * 
 * If the controller is under XR-3 mode, all motors are affected. No motor may be
 * executing a trapezoidal move. This can be checked by using either the SA or SS
 * command.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_mi(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		mcuicom_send("MI" CMDEND);
	}
}

/**
 * Stop Single Motor.
 * 
 * Stops the specified motor regardless of motor or controller mode.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_mm(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if ('A' <= param1.value.letter && param1.value.letter <= 'F')
				{
					int size = 64;
					char buf[size];
					
					// Send MCUICOM command SA, SB, ..., SF depending on motor letter (param 1)
					buf[0] = 'S';
					buf[1] = param1.value.letter;
					buf[2] = *CMDEND;
					mcuicom_send(buf);
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be a letter
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Start Single Motor.
 * 
 * Moves the specified motor to its destination position under its set motor
 * velocity.
 * 
 * The motor must be in trapezoidal mode with a non-zero velocity. If the
 * relative destination register is non-zero, a relative move will be made.
 * If the relative destination register is zero and the destination register
 * is the same as the current position, then no movement will take place.
 * 
 * The specified motor must not be currently executing a trapezoidal move.
 * This can be checked by issuing an SA command.
 * 
 * This command is invalid for motor port A if it is enabled as the gripper.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ms(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				switch (param1.value.letter)
				{
					case 'A':
						if (gripper_is_enabled())
						{
							// error: port A is enabled as the gripper
							dbgmsg_uart2(ERR_PORT_A_IS_GRIPPER);
						}
						else
						{
							// proceed
						}
						break;
					case 'B':
						break;
					case 'C':
						break;
					case 'D':
						break;
					case 'E':
						break;
					case 'F':
						break;
					case 'G':
						break;
					case 'H':
						break;
					default:
						// error: parameter 1 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be a letter
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Start an XYZ Move.
 * 
 * Move to the predefined XYZ position in a coordinated fashion.
 * 
 * The controller must be configured for the XR-3 or SCARA robot types.
 * 
 * If the XR-3 is used, motors B through F must be configured for trapezoidal
 * mode and have a non-zero desired motor velocity.
 * 
 * If the SCARA is used, motors B through E must be configured for trapezoidal
 * mode and have a non-zero desired motor velocity.
 * 
 * A hard home must have been previously executed.
 * 
 * If the controller is under XR-3 mode, motors B through F must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * If the controller is under SCARA mode, motors B through E must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * If, on issuance of the MX command, the desired position is in bounds, the robot
 * will move to the new position in a coordinated fashion. Any axis whose relative
 * xyz destination position is non-zero will make a relative movement and the relative
 * destination register will become zero.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_mx(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (hard_home_executed())
		{
			if (controller_mode() == XR3)
			{
				if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E | MOTOR_F))
				{
					// error: no motor can be executing a trapezoidal move
					dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
				}
				else
				{
					// proceed
				}
			}
			else if (controller_mode() == SCARA)
			{
				if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E))
				{
					// error: no motor can be executing a trapezoidal move
					dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
				}
				else
				{
					// proceed
				}
			}
		}
		else
		{
			// error: a hard home must have been previously executed
		}
	}
}

/**
 * Set Destination Position, Absolute.
 * 
 * Sets the desired position a motor in trapezoidal mode will move to in encoder counts.
 * 
 * If the specified motor is in trapezoidal mode, it must not be currently executing a
 * trapezoidal move. This can be checked by issuing the SA command.
 * 
 * If the relative destination register is non-zero, then on issuance of a move command
 * the destination register will be overwritten.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_pd(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if ('A' <= param1.value.letter && param1.value.letter <= 'F')
				{
					char motor = 1 << (param1.value.letter - 'A');
					if (any_motor_executing_trapezoidal_move(motor))
					{
						// error: if the motor is in trapezoidal mode, it must not be executing a trapezoidal move
						dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
					}
					else
					{
						if (param2.present)
						{
							if (param2.type == TOKEN_INT)
							{
								int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
								if (-32767 <= intparam2 && intparam2 <= 32767)
								{
									const int size = 64;
									char buf[size];
									
									// Send MCUICOM command AA, AB, ..., AF depending on motor letter (param 1)
									snprintf(buf, size, "A%c,%d%c", param1.value.letter, intparam2, *CMDEND);
									mcuicom_send(buf);
								}
								else
								{
									// error: parameter 2 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
								}
							}
							else
							{
								// error: parameter 2 must be an integer number
								dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
							}
						}
						else
						{
							// error: parameter 2 must be specified
							dbgmsg_uart2(ERR_MISSING_PARAMS);
						}
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Destination Position, Relative.
 * 
 * Sets the relative movement a motor in trapezoidal mode will travel. Motor desired
 * position becomes the old desired position plus the relative movement.
 * 
 * If the specified motor is in trapezoidal mode, it must not be currently executing a
 * trapezoidal move. This can be checked by issuing the SA command.
 * 
 * After issuance of amove command, the relative destination register will become zero.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_pr(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if ('A' <= param1.value.letter && param1.value.letter <= 'F')
				{
					char motor = 1 << (param1.value.letter - 'A');
					if (any_motor_executing_trapezoidal_move(motor))
					{
						// error: if the motor is in trapezoidal mode, it must not be executing a trapezoidal move
						dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
					}
					else
					{
						if (param2.present)
						{
							if (param2.type == TOKEN_INT)
							{
								int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
								if (-32767 <= intparam2 && intparam2 <= 32767)
								{
									const int size = 64;
									char buf[size];
									
									// Send MCUICOM command BA, BB, ..., BF depending on motor letter (param 1)
									snprintf(buf, size, "B%c,%d%c", param1.value.letter, intparam2, *CMDEND);
									mcuicom_send(buf);
								}
								else
								{
									// error: parameter 2 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
								}
							}
							else
							{
								// error: parameter 2 must be an integer number
								dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
							}
						}
						else
						{
							// error: parameter 2 must be specified
							dbgmsg_uart2(ERR_MISSING_PARAMS);
						}
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set XYZ Destination, Absolute.
 * 
 * Sets the desired position an X, Y or Z axis will move to in millimeters or the
 * angle the A or T axis will move to in degrees.
 * 
 * If the controller is under XR-3 mode, motors B through F must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * If the controller is under SCARA mode, motors B through E must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * If the relative destination register is non-zero, then on issuance of a move
 * command the destination register will be overwritten.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_px(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		bool_t error = false;
		if (controller_mode() == XR3)
		{
			if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E | MOTOR_F))
			{
				error = true;
			}
		}
		else if (controller_mode() == SCARA)
		{
			if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E))
			{
				error = true;
			}
		}
		
		if (!error)
		{
			if (param1.present)
			{
				if (param1.type == TOKEN_LETTER)
				{
					if ('X' <= param1.value.letter && param1.value.letter <= 'Z')
					{
						if (param2.present)
						{
							if (param2.type == TOKEN_DEC)
							{
								float floatparam2 = param2.value.decimal.int_part + param2.value.decimal.dec_part / 100.0;
								if (-1000.00 <= floatparam2 && floatparam2 <= 1000.00)
								{
									const int size = 64;
									char buf[size];
									snprintf(buf, size, "C%c,%.2f%c", param1.value.letter, (double) floatparam2, *CMDEND);
									mcuicom_send(buf);
								}
								else
								{
									// error: parameter 2 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
								}
							}
							else
							{
								// error: parameter 2 must be a decimal number
								dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
							}
						}
						else
						{
							// error: parameter 2 must be specified
							dbgmsg_uart2(ERR_MISSING_PARAMS);
						}
					}
					else
					{
						// error: parameter 1 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error parameter 1 must be a letter
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 1 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
	}
}

/**
 * Set XYZ Destination, Relative.
 * 
 * Sets the desired position an X, Y or Z axis will move to in millimeters or the
 * angle the A or T axis will move to in degrees.
 * 
 * If the controller is under XR-3 mode, motors B through F must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * If the controller is under SCARA mode, motors B through E must not be executing
 * a trapezoidal move and can be checked using the SA command.
 * 
 * After issuance of a move command, the relative destination register will become
 * zero.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_py(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		bool_t error = false;
		if (controller_mode() == XR3)
		{
			if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E | MOTOR_F))
			{
				error = true;
			}
		}
		else if (controller_mode() == SCARA)
		{
			if (any_motor_executing_trapezoidal_move(MOTOR_B | MOTOR_C | MOTOR_D | MOTOR_E))
			{
				error = true;
			}
		}
		
		if (!error)
		{
			if (param1.present)
			{
				if (param1.type == TOKEN_LETTER)
				{
					if ('X' <= param1.value.letter && param1.value.letter <= 'Z')
					{
						if (param2.present)
						{
							if (param2.type == TOKEN_DEC)
							{
								float floatparam2 = param2.value.decimal.int_part + param2.value.decimal.dec_part / 100.0;
								if (-1000.00 <= floatparam2 && floatparam2 <= 1000.00)
								{
									const int size = 64;
									char buf[size];
									snprintf(buf, size, "D%c,%.2f%c", param1.value.letter, (double) floatparam2, *CMDEND);
									mcuicom_send(buf);
								}
								else
								{
									// error: parameter 2 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
								}
							}
							else
							{
								// error: parameter 2 must be a decimal number
								dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
							}
						}
						else
						{
							// error: parameter 2 must be specified
							dbgmsg_uart2(ERR_MISSING_PARAMS);
						}
					}
					else
					{
						// error: parameter 1 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error parameter 1 must be a letter
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 1 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
	}
}

/**
 * Set System Velocity.
 * 
 * Set system velocity as a percentage of system maximum velocity.
 * 
 * System velocity is a global parameter that affects all motors. If system velocity
 * is 0, all motors will be stopped.
 * 
 * Motors in trapezoidal mode must not be currently executing a trapezoidal move. This
 * can be checked by issuing the SS command. The gripper cannot be in the process of
 * closing or opening.
 * 
 * Motors in velocity mode will immediately begin tracking their new velocities.
 * 
 * This command cannot be used while in teach pendant mode.
 */
inline void hostcmd_vg(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (0 <= intparam1 && intparam1 <= 100)
				{
					if (any_motor_executing_trapezoidal_move(MOTOR_ALL))
					{
						dbgmsg_uart2(ERR_TRAPEZOIDAL_MOVE);
					}
					else
					{
						const int size = 64;
						char buf[size];

						snprintf(buf, size, "SV,%d" CMDEND, intparam1);
						mcuicom_send(buf);

						// FIXME: the gripper must not be in the process of
						// closing or opening
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Motor Velocity.
 * 
 * Set motor velocity as a percentage of system velocity.
 * 
 * If the specified motor is in trapezoidal mode, it must not be currently executing
 * a trapezoidal move. This can be checked by issuing the SA command. The sign or
 * direction of velocity has no meaning under trapezoidal mode.
 * 
 * If the specified motor is in velocity mode, the motor will immediately begin tracking
 * the new velocity. A minus sign indicates negative movement.
 * 
 * If the specified motor is in idle or open loop mode, the motor desired velocity will be
 * set but no motion will take place.
 * 
 * If system velocity is 0, no motor movement will take place.
 * 
 * Whenever the gripper is enabled or commanded to open or close, a factory set velocity
 * will be used for the gripper. The programmed motor velocity will have no meaning.
 * 
 * This command cannot be used while in teach pendant mode.
 */
inline void hostcmd_vs(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if (param2.present)
				{
					if (param2.type == TOKEN_INT)
					{
						int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
						if (-100 <= intparam2 && intparam2 <= 100)
						{
							switch (param1.value.letter)
							{
								case 'A':
									break;
								case 'B':
									break;
								case 'C':
									break;
								case 'D':
									break;
								case 'E':
									break;
								case 'F':
									break;
								case 'G':
									break;
								case 'H':
									break;
								default:
									// error: parameter 1 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 must be an integer number
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error: parameter 2 must be specified
					dbgmsg_uart2(ERR_MISSING_PARAMS);
				}
			}
			else
			{
				// error: parameter 1 must be a letter
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set XYZ Rotation Angle.
 * 
 * Sets the angle of rotation of the user's coordinate system with respect to the
 * robot coordinate system.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xa(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_DEC)
			{
				float floatparam1 = param1.value.decimal.int_part + param1.value.decimal.dec_part / 100.0;
				if (-1000.00 <= floatparam1 && floatparam1 <= 1000.00)
				{
					// proceed
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be a decimal number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
		}
	}
}

/**
 * Set XYZ Home Position.
 * 
 * Sets the linear distance between the robot coordinate system origin and the center
 * of the tool tip (gripper).
 * 
 * X, Y and Z are in units of millimeters, and A and T are in units of degrees.
 * 
 * The A axis does not exist on the SCARA robot.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xh(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if (param2.present)
				{
					if (param2.type == TOKEN_DEC)
					{
						float floatparam2 = param2.value.decimal.int_part + param2.value.decimal.dec_part / 100.0;
						if (-1000.00 <= floatparam2 && floatparam2 <= 1000.00)
						{
							switch (param1.value.letter)
							{
								case 'A':
									break;
								case 'B':
									break;
								case 'C':
									break;
								case 'D':
									break;
								case 'E':
									break;
								case 'F':
									break;
								case 'G':
									break;
								case 'H':
									break;
								default:
									// error: parameter 1 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 must be a decimal number
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error: parameter 2 must be specified
					dbgmsg_uart2(ERR_MISSING_PARAMS);
				}
			}
			else
			{
				// error: parameter 1 must be a letter
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set the XYZ Offset.
 * 
 * Sets the linear or angular displacement between the user coordinate system and
 * the robot coordinate system.
 * 
 * X, Y and Z are in units of millimeters, and A and T are in units of degrees.
 * 
 * The A axis does not exist on the SCARA robot.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xo(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_LETTER)
			{
				if (param2.present)
				{
					if (param2.type == TOKEN_DEC)
					{
						float floatparam2 = param2.value.decimal.int_part + param2.value.decimal.dec_part / 100.0;
						if (-1000.00 <= floatparam2 && floatparam2 <= 1000.00)
						{
							switch (param1.value.letter)
							{
								case 'A':
									break;
								case 'B':
									break;
								case 'C':
									break;
								case 'D':
									break;
								case 'E':
									break;
								case 'F':
									break;
								case 'G':
									break;
								case 'H':
									break;
								default:
									// error: parameter 1 out of range
									dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 must be a decimal number
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error: parameter 2 must be specified
					dbgmsg_uart2(ERR_MISSING_PARAMS);
				}
			}
			else
			{
				// error: parameter 1 must be a letter
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Aux Port Leverl and Direction.
 * 
 * Sets an auxiliary port's PWM level and direction.
 * 
 * PWM level is a percentage of the absolute value of maximum motor power. A minus
 * sign indicates negative voltage or direction.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xs(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				if (param2.present)
				{
					if (param2.type == TOKEN_INT)
					{
						int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
						if (-100 <= intparam2 && intparam2 <= 100)
						{
							int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
							if (intparam1 == 1)
							{
								// proceed
							}
							else if (intparam1 == 2)
							{
								// proceed
							}
							else
							{
								// error: parameter 1 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 out of range
							dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 must be an integer number
						dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
					}
				}
				else
				{
					// error: parameter 2 must be specified
					dbgmsg_uart2(ERR_MISSING_PARAMS);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Tool Length.
 * 
 * Sets the distance from the hand flex axis to the tool tip (gripper end).
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xt(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_DEC)
			{
				float floatparam1 = param1.value.decimal.int_part + param1.value.decimal.dec_part / 100.0;
				if (-1000.00 <= floatparam1 && floatparam1 <= 1000.00)
				{
					// proceed
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be a decimal number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Height of Elbow Rotation Axis.
 * 
 * Sets the height of the elbow rotation axis from the reference surface.
 * 
 * This parameter has no meaning if the current robot type is SCARA.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_xy(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_DEC)
			{
				float floatparam1 = param1.value.decimal.int_part + param1.value.decimal.dec_part / 100.0;
				if (-1000.00 <= floatparam1 && floatparam1 <= 1000.00)
				{
					// proceed
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be a decimal number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Receive Teach Pendant File from Host.
 */
inline void hostcmd_fr(void)
{
}

/**
 * Transmit Teach Pendant File to Host.
 */
inline void hostcmd_ft(void)
{
}

/**
 * Execute Teach Pendant Program.
 * 
 * Begin execution of a teach pendant program.
 * 
 * The controller must be under teach pendant control and in play mode. Issuing an
 * FX command is the same as pressing the RUN key on the pendant. The pendant will
 * display any appropriate error messages such as NO PROGRAM or NO_HARD, which means
 * a hard home is required. The host computer cannot halt or suspend the program.
 * Issuing a second FX while a program is executing will result in an error, since
 * the pendant is no longer in play mode.
 */
inline void hostcmd_fx(void)
{
}

/**
 * Execute Teach Pendant Program.
 * 
 * Terminate execution of a teach pendant program.
 * 
 * The controller must be under teach pendant control and in play mode. Issuing a
 * TA command is the same as pressing the ABORT key on the pendant.
 */
inline void hostcmd_ta(void)
{
}

/**
 * Clear Teach Pendant Display.
 * 
 * Clears the teach pendant display and sets the cursos to the home position.
 * The cursor is set to the top row, leftmost character.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_tc(void)
{
}

/**
 * Print to Teach Pendant Display.
 * 
 * Prints a message to the pendant display at the current cursor position.
 * 
 * The message must be less than or equal to 16 characters. If a space, tab
 * or punctuation mark is in the message, the message must be delimited by
 * double quote marks.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_td(void)
{
}

/**
 * Enable/Disable Teach Pendant to Move Motors.
 * 
 * Under host computer control, allow or disallow the pendant to move motors.
 * 
 * This command allows a host computer application to let the pendant take control
 * over moving the motors. After the host computer detects an ENTER or ESCAPE key,
 * the host can read the various motor positions and store them as a point for later
 * replay.
 * 
 * A value of d = 0 disables the pendant, while a value of d = 1 allows the pendant to
 * take temporary control.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_te(void)
{
}

/**
 * Give Control to Host.
 * 
 * Toggles system control between the host computer and the teach pendant.
 * 
 * For the host computer to take full control over the controller, the teach pendant
 * must be in the PLAY mode. There is no effect if the controller is already under host
 * control or if no teach pendant is connected.
 */
inline void hostcmd_th(void)
{
	// Set bit 7 of the 'system_config' register, which indicates whether the system
	// is in host mode or teach pendant mode. A value of 1 indicates host mode.
	controller.system_config |= BIT_7;
}

/**
 * Give Control to Teach Pendant.
 * 
 * Toggles system control between the host computer and the teach pendant.
 * 
 * A teach pendant must be attached in order for the host computer to give control away.
 * There is no effect if the controller is already under teach pendant control.
 */
inline void hostcmd_tx(void)
{
	// Clear bit 7 of the 'system_config' register, which indicates whether the system
	// is in host mode or teach pendant mode. A value of 0 indicates teach pendant mode.
	controller.system_config &= 0x7F;
}

/**
 * Return to Host the Next Key Code.
 * 
 * Returns the next key pressed on the pendant.
 * 
 * Waits until a key is pressed and returns the associated key code.
 */
inline void hostcmd_tk(void)
{
}

/**
 * Return to Host the Last Key Code.
 * 
 * Returns the last key pressed on the pendant.
 * 
 * Returns the code of the last key pressed.
 */
inline void hostcmd_tl(void)
{
}

/**
 * Reset the Teach Pendant.
 * 
 * Resets the teach pendant and clears the display.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_tr(void)
{
}

/**
 * Set Teach Pendant Display Cursor.
 * 
 * Sets the cursor position on the pendant's LCD display.
 * 
 * This command is used prior to issuing a TD (print message) command in order to
 * place the message at the desired location within the display.
 * 
 * A teach pendant must be connected to the controller.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ts(void)
{
}

/**
 * Execute Teach Pendant Diagnostics and Return Results.
 * 
 * Causes the teach pendant to test itself and returns the results.
 * 
 * The controller responds with 0 (zero) if the teach pendant is functioning properly
 * and E or a number if the teach pendant detects and error.
 * 
 * If a number is returned, it will represent the location of a struck or held key.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_tt(void)
{
}

/**
 * Set Proportional Gain.
 * 
 * Set the proportional gain multiplier for the specified motor for the current robot
 * type.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_ka(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if (param2.present)
			{
				if (param2.type == TOKEN_INT)
				{
					int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
					if (0 <= intparam2 && intparam2 <= 255)
					{
						switch (param1.value.letter)
						{
							case 'A':
								break;
							case 'B':
								break;
							case 'C':
								break;
							case 'D':
								break;
							case 'E':
								break;
							case 'F':
								break;
							case 'G':
								break;
							case 'H':
								break;
							default:
								// error: parameter 1 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error: parameter 2 must be an integer number
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 2 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Set Differential Gain.
 * 
 * Set the differential gain multiplier for the specified motor for the current robot
 * type.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_kb(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if (param2.present)
			{
				if (param2.type == TOKEN_INT)
				{
					int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
					if (0 <= intparam2 && intparam2 <= 255)
					{
						switch (param1.value.letter)
						{
							case 'A':
								break;
							case 'B':
								break;
							case 'C':
								break;
							case 'D':
								break;
							case 'E':
								break;
							case 'F':
								break;
							case 'G':
								break;
							case 'H':
								break;
							default:
								// error: parameter 1 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error: parameter 2 must be an integer number
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 2 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Set Integral Gain.
 * 
 * Set the integral gain multiplier for the specified motor for the current robot
 * type.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_kc(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			if (param2.present)
			{
				if (param2.type == TOKEN_INT)
				{
					int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
					if (0 <= intparam2 && intparam2 <= 255)
					{
						switch (param1.value.letter)
						{
							case 'A':
								break;
							case 'B':
								break;
							case 'C':
								break;
							case 'D':
								break;
							case 'E':
								break;
							case 'F':
								break;
							case 'G':
								break;
							case 'H':
								break;
							default:
								// error: parameter 1 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
						}
					}
					else
					{
						// error: parameter 2 out of range
						dbgmsg_uart2(ERR_OUT_OF_RANGE);
					}
				}
				else
				{
					// error: parameter 2 must be an integer number
					dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
				}
			}
			else
			{
				// error: parameter 2 must be specified
				dbgmsg_uart2(ERR_MISSING_PARAMS);
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Proportional Gain.
 * 
 * Return the proportional gain multiplier for the specified motor for the current robot
 * type.
 * 
 * The value returned is in the range of 0 to 255.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_ra(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'A':
					break;
				case 'B':
					break;
				case 'C':
					break;
				case 'D':
					break;
				case 'E':
					break;
				case 'F':
					break;
				case 'G':
					break;
				case 'H':
					break;
				default:
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					break;
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Differential Gain.
 * 
 * Return the differential gain multiplier for the specified motor for the current robot
 * type.
 * 
 * The value returned is in the range of 0 to 255.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_rb(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'A':
					break;
				case 'B':
					break;
				case 'C':
					break;
				case 'D':
					break;
				case 'E':
					break;
				case 'F':
					break;
				case 'G':
					break;
				case 'H':
					break;
				default:
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					break;
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Integral Gain.
 * 
 * Return the integral gain multiplier for the specified motor for the current robot
 * type.
 * 
 * The value returned is in the range of 0 to 255.
 * 
 * If robot type is changed, the gains will be reset.
 * 
 * This command can be used while udner teach pendant mode.
 */
inline void hostcmd_rc(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_LETTER)
		{
			switch (param1.value.letter)
			{
				case 'A':
					break;
				case 'B':
					break;
				case 'C':
					break;
				case 'D':
					break;
				case 'E':
					break;
				case 'F':
					break;
				case 'G':
					break;
				case 'H':
					break;
				default:
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
					break;
			}
		}
		else
		{
			// error: parameter 1 must be a letter
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Restore User Gains from EEPROM.
 * 
 * Resets the gains previously stored by the user from EEPROM for all motors for the current
 * robot type.
 * 
 * The desired robot type should be set before issuing the KR command.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_kr(void)
{
}

/**
 * Store User Gains from EEPROM.
 * 
 * Stores the gains for all motors for the current robot type to EEPROM.
 * 
 * If the gains have been modified, KS should be issued before changing the robot types or
 * shutting the system down.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_ks(void)
{
}

/**
 * Restore Factory Gains.
 * 
 * Resets the gains stored by the factory for all motors for the current robot type.
 * 
 * The desired robot type should be set before issuing a KX command.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_kx(void)
{
}

/**
 * Read Input or Switch Bit
 * 
 * Returns the state of the specified input or switch bit.
 * 
 * A bit specifier of 1 to 8 addresses input bits 1 to 8, while a bit specifier of 9 to 16
 * addresses switch bits 1 to 8, respectively.
 * 
 * A returned value of 1 indicates the input is on or the switch is closed.
 * A returned value of 0 indicates the input is off or the switch is open.
 * 
 * An input is considered on if current is flowing through the input port.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_ib(void)
{
	if (param1.present)
	{
		if (param1.type == TOKEN_INT)
		{
			int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
			if (1 <= intparam1 && intparam1 <= 16)
			{
				// proceed
			}
			else
			{
				// error: parameter 1 out of range
				dbgmsg_uart2(ERR_OUT_OF_RANGE);
			}
		}
		else
		{
			// error: parameter 1 must be an integer number
			dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
		}
	}
	else
	{
		// error: parameter 1 must be specified
		dbgmsg_uart2(ERR_MISSING_PARAMS);
	}
}

/**
 * Read Input Port.
 * 
 * Returns the state of all eight input bits.
 * 
 * The value returned is the decimal representation of the byte and ranges from 0 to 255.
 * The value returned must be decoded to determine the state of each input.
 * 
 * A bit value of 1 indicates the corresponding input is on.
 * A bit value of 0 indicates the corresponding input is off.
 * 
 * An input is considered on if current is flowing through the input port.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_ip(void)
{
}

/**
 * Read Switch Port.
 * 
 * Returns the state of all eight switch input bits.
 * 
 * The value returned is the decimal representation of the byte and ranges from 0 to 255.
 * The value returned must be decoded to determine the state of each input.
 * 
 * A bit value of 1 indicates the corresponding input is on.
 * A bit value of 0 indicates the corresponding input is off.
 * 
 * An input is considered on if current is flowing through the input port.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_ix(void)
{
}

/**
 * Set Output Bit.
 * 
 * Turn on or off an output port.
 * 
 * If s = 1, the specified output is turned on.
 * If s = 0, the specified output is turned off.
 * 
 * Turning on an output allows current to flow through the port.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ob(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (1 <= intparam1 && intparam1 <= 8)
				{
					if (param2.present)
					{
						if (param2.type == TOKEN_INT)
						{
							int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
							if (intparam2 == 0 || intparam2 == 1)
							{
								// proceed
							}
							else
							{
								// error: parameter 2 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 must be an integer number
							dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
						}
					}
					else
					{
						// error: parameter 2 must be specified
						dbgmsg_uart2(ERR_MISSING_PARAMS);
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Set Output Port.
 * 
 * Set the output port to the hexadecimal representation of the value sent.
 * 
 * The value to be sent is the decimal representation of the byte to be written.
 * Output port number 1 is the least significant bit.
 * 
 * Turning on an output allows current to flow through the output.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_op(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (0 <= intparam1 && intparam1 <= 255)
				{
					// proceed
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Read Output Port.
 * 
 * Returns the state of all eight output bits.
 * 
 * The value returned is the decimal representation of the byte and ranges from 0 to 255.
 * The value returned must be decoded to determine the state of each output.
 * 
 * Output port number 1 is the least significant bit.
 * 
 * A bit value of 1 indicates the corresponding output is on.
 * A bit value of 0 indicates the corresponding output is off.
 * 
 * The returned value represents the last state the outputs were set to and not the actual state
 * of the output.
 * 
 * If an output is on, it allows current to flow.
 * 
 * This command can be used while under teach pendant mode.
 */
inline void hostcmd_or(void)
{
}

/**
 * Toggle Output Bit.
 * 
 * Toggle the specified output port to the specified state.
 * 
 * If the specified output is the complement of the desired state, the output will equal the desired
 * statue for 1/60 second then return to the original state. If the specified output is already in the
 * desired state, the output will be complemented.
 * 
 * Turing on an output allows current to flow through the output.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_ot(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (1 <= intparam1 && intparam1 <= 8)
				{
					if (param2.present)
					{
						if (param2.type == TOKEN_INT)
						{
							int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
							if (intparam2 == 0 || intparam2 == 1)
							{
								// proceed
							}
							else
							{
								// error: parameter 2 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 must be an integer number
							dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
						}
					}
					else
					{
						// error: parameter 2 must be specified
						dbgmsg_uart2(ERR_MISSING_PARAMS);
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}

/**
 * Abort all Waits.
 * 
 * Removes all pending wait on inputs and wait on switches.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_wa(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		// proceed
	}
}

/**
 * Wait On Input or Switch.
 * 
 * Detect when the specified input or switch matches the specified state.
 * 
 * A bit specifier of 1 to 8 addresses input bits 1 to 8, while a bit specifier of 9 to 16
 * addresses switch bits 1 to 8, respectively.
 * 
 * The host must poll the system status byte (bit 4) to determine when all wait on inputs
 * have matched. This is different from the teach pendant system, which waits until the
 * state is matched.
 * 
 * In the controller, each input bit has a corresponding wait flag. When a WI command is
 * received, the flag corresponding to the specified input bit is set if the specified
 * state does not match the actual state of the input. The flag is cleared if the actual
 * state of the specified input bit matches the specified state. The input is continually
 * polled until a match occurs or the WA (abort all waits) command is issued.
 * 
 * The system status byte (bit 4) is set if any wait on inputs are still pending and cleared
 * if there are no wait on inputs pending.
 * 
 * This command cannot be used while under teach pendant mode.
 */
inline void hostcmd_wi(void)
{
	if (controller_is_in_teach_pendant_mode())
	{
		// error: command cannot be used while under teach pendant mode
		dbgmsg_uart2(ERR_TEACH_PENDANT_MODE);
	}
	else
	{
		if (param1.present)
		{
			if (param1.type == TOKEN_INT)
			{
				int intparam1 = param1.value.integer.sign * param1.value.integer.abs_value;
				if (1 <= intparam1 && intparam1 <= 16)
				{
					if (param2.present)
					{
						if (param2.type == TOKEN_INT)
						{
							int intparam2 = param2.value.integer.sign * param2.value.integer.abs_value;
							if (intparam2 == 0 || intparam2 == 1)
							{
								// proceed
							}
							else
							{
								// error: parameter 2 out of range
								dbgmsg_uart2(ERR_OUT_OF_RANGE);
							}
						}
						else
						{
							// error: parameter 2 must be an integer number
							dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
						}
					}
					else
					{
						// error: parameter 2 must be specified
						dbgmsg_uart2(ERR_MISSING_PARAMS);
					}
				}
				else
				{
					// error: parameter 1 out of range
					dbgmsg_uart2(ERR_OUT_OF_RANGE);
				}
			}
			else
			{
				// error: parameter 1 must be an integer number
				dbgmsg_uart2(ERR_WRONG_TYPE_PARAM);
			}
		}
		else
		{
			// error: parameter 1 must be specified
			dbgmsg_uart2(ERR_MISSING_PARAMS);
		}
	}
}
