#include "shell.h"
#include "hostcom.h"
#include "../types.h"
#include "../hostcmdset.h"

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

void hostcmd_sa(void);
void hostcmd_sc(void);
void hostcmd_sd(void);
void hostcmd_se(void);
void hostcmd_sm(void);
void hostcmd_sp(void);
void hostcmd_sr(void);
void hostcmd_ss(void);
void hostcmd_st(void);
void hostcmd_su(void);
void hostcmd_sv(void);
void hostcmd_sx(void);
void hostcmd_sz(void);
void hostcmd_cc(void);
void hostcmd_cg(void);
void hostcmd_cm(void);
void hostcmd_cr(void);
void hostcmd_ar(void);
void hostcmd_dr(void);
void hostcmd_gs(void);
void hostcmd_hr(void);
void hostcmd_pa(void);
void hostcmd_pw(void);
void hostcmd_pz(void);
void hostcmd_rl(void);
void hostcmd_ua(void);
void hostcmd_uh(void);
void hostcmd_uo(void);
void hostcmd_ut(void);
void hostcmd_uy(void);
void hostcmd_va(void);
void hostcmd_vr(void);
void hostcmd_vx(void);
void hostcmd_xr(void);
void hostcmd_ac(void);
void hostcmd_as(void);
void hostcmd_ds(void);
void hostcmd_gc(void);
void hostcmd_go(void);
void hostcmd_ha(void);
void hostcmd_hg(void);
void hostcmd_hh(void);
void hostcmd_hl(void);
void hostcmd_hs(void);
void hostcmd_ma(void);
void hostcmd_mc(void);
void hostcmd_mi(void);
void hostcmd_mm(void);
void hostcmd_ms(void);
void hostcmd_mx(void);
void hostcmd_pd(void);
void hostcmd_pr(void);
void hostcmd_px(void);
void hostcmd_py(void);
void hostcmd_vg(void);
void hostcmd_vs(void);
void hostcmd_xa(void);
void hostcmd_xh(void);
void hostcmd_xo(void);
void hostcmd_xs(void);
void hostcmd_xt(void);
void hostcmd_xy(void);
void hostcmd_fr(void);
void hostcmd_ft(void);
void hostcmd_fx(void);
void hostcmd_ta(void);
void hostcmd_tc(void);
void hostcmd_td(void);
void hostcmd_te(void);
void hostcmd_th(void);
void hostcmd_tx(void);
void hostcmd_tk(void);
void hostcmd_tl(void);
void hostcmd_tr(void);
void hostcmd_ts(void);
void hostcmd_tt(void);
void hostcmd_ka(void);
void hostcmd_kb(void);
void hostcmd_kc(void);
void hostcmd_ra(void);
void hostcmd_rb(void);
void hostcmd_rc(void);
void hostcmd_kr(void);
void hostcmd_ks(void);
void hostcmd_kx(void);
void hostcmd_ib(void);
void hostcmd_ip(void);
void hostcmd_ix(void);
void hostcmd_ob(void);
void hostcmd_op(void);
void hostcmd_or(void);
void hostcmd_ot(void);
void hostcmd_wa(void);
void hostcmd_wi(void);

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
	cmd_buf_pos = 0;
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
			
			if (cmd_buf[cmd_buf_pos++] == '-')
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
					break;
			}
			break;
		default:
			break;
	}
}

/******************************************************************************
 *                    HOST COMMAND FUNCTION IMPLEMENTATION                    *
 ******************************************************************************/

void hostcmd_sa(void)
{
}

void hostcmd_sc(void)
{
}

void hostcmd_sd(void)
{
}

void hostcmd_se(void)
{
}

void hostcmd_sm(void)
{
}

void hostcmd_sp(void)
{
}

void hostcmd_sr(void)
{
}

void hostcmd_ss(void)
{
}

void hostcmd_st(void)
{
}

void hostcmd_su(void)
{
}

void hostcmd_sv(void)
{
}

void hostcmd_sx(void)
{
}

void hostcmd_sz(void)
{
}

void hostcmd_cc(void)
{
}

void hostcmd_cg(void)
{
}

void hostcmd_cm(void)
{
}

void hostcmd_cr(void)
{
}

void hostcmd_ar(void)
{
}

void hostcmd_dr(void)
{
}

void hostcmd_gs(void)
{
}

void hostcmd_hr(void)
{
}

void hostcmd_pa(void)
{
}

void hostcmd_pw(void)
{
}

void hostcmd_pz(void)
{
}

void hostcmd_rl(void)
{
}

void hostcmd_ua(void)
{
}

void hostcmd_uh(void)
{
}

void hostcmd_uo(void)
{
}

void hostcmd_ut(void)
{
}

void hostcmd_uy(void)
{
}

void hostcmd_va(void)
{
}

void hostcmd_vr(void)
{
}

void hostcmd_vx(void)
{
}

void hostcmd_xr(void)
{
}

void hostcmd_ac(void)
{
}

void hostcmd_as(void)
{
}

void hostcmd_ds(void)
{
}

void hostcmd_gc(void)
{
}

void hostcmd_go(void)
{
}

void hostcmd_ha(void)
{
}

void hostcmd_hg(void)
{
}

void hostcmd_hh(void)
{
}

void hostcmd_hl(void)
{
}

void hostcmd_hs(void)
{
}

void hostcmd_ma(void)
{
}

void hostcmd_mc(void)
{
}

void hostcmd_mi(void)
{
}

void hostcmd_mm(void)
{
}

void hostcmd_ms(void)
{
}

void hostcmd_mx(void)
{
}

void hostcmd_pd(void)
{
}

void hostcmd_pr(void)
{
}

void hostcmd_px(void)
{
}

void hostcmd_py(void)
{
}

void hostcmd_vg(void)
{
}

void hostcmd_vs(void)
{
}

void hostcmd_xa(void)
{
}

void hostcmd_xh(void)
{
}

void hostcmd_xo(void)
{
}

void hostcmd_xs(void)
{
}

void hostcmd_xt(void)
{
}

void hostcmd_xy(void)
{
}

void hostcmd_fr(void)
{
}

void hostcmd_ft(void)
{
}

void hostcmd_fx(void)
{
}

void hostcmd_ta(void)
{
}

void hostcmd_tc(void)
{
}

void hostcmd_td(void)
{
}

void hostcmd_te(void)
{
}

void hostcmd_th(void)
{
}

void hostcmd_tx(void)
{
}

void hostcmd_tk(void)
{
}

void hostcmd_tl(void)
{
}

void hostcmd_tr(void)
{
}

void hostcmd_ts(void)
{
}

void hostcmd_tt(void)
{
}

void hostcmd_ka(void)
{
}

void hostcmd_kb(void)
{
}

void hostcmd_kc(void)
{
}

void hostcmd_ra(void)
{
}

void hostcmd_rb(void)
{
}

void hostcmd_rc(void)
{
}

void hostcmd_kr(void)
{
}

void hostcmd_ks(void)
{
}

void hostcmd_kx(void)
{
}

void hostcmd_ib(void)
{
}

void hostcmd_ip(void)
{
}

void hostcmd_ix(void)
{
}

void hostcmd_ob(void)
{
}

void hostcmd_op(void)
{
}

void hostcmd_or(void)
{
}

void hostcmd_ot(void)
{
}

void hostcmd_wa(void)
{
}

void hostcmd_wi(void)
{
}
