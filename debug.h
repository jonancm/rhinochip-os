#ifndef DEBUG_H
#define DEBUG_H

#define NDEBUG

#ifndef NDEBUG
void dbgmsg_uart1(char *c);
#else
#define dbgmsg_uart1(x)    /* Define empty macro */
#endif

#ifndef NDEBUG
void dbgmsg_uart2(char *c);
#else
#define dbgmsg_uart2(x)    /* Define empty macro */
#endif

#endif
