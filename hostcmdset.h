#ifndef HOSTCMDSET_H
#define HOSTCMDSET_H

/******************************************************************************
 * SYNTAX FOR COMMANDS
 ******************************************************************************/

// character that separates the command name from its arguments

#define ARGSEP    ","

/******************************************************************************
 * SYNTAX FOR PROGRAM FILES
 ******************************************************************************/

// start of program file

#define FILESTART    "\x30\x30\x31\x31\x31\x31\x31\x31\x31\x31\x30"

// end of program file

#define FILEEND    "\x25\x1a"

// command separator in program files

#define CMDSEP    "\n"

/******************************************************************************
 * SYNTAX FOR TERMINAL
 ******************************************************************************/

// end of command in terminal mode

#define CMDEND    "\r" // command separator (end of command)

#endif
