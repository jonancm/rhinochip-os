#ifndef MCTLCOM_H
#define MCTLCOM_H

#include "../mcuicom.h"

/**
 * mctlcom timeout in milliseconds
 * Default value: 0.100 seconds (100 ms)
 * Recommended range (at 29.50 MHz with 1:64 prescaler): 1 ms ~ 100 ms
 */
#define MCTLCOM_TIMEOUT    100

inline void mctlcom_setup(void);
int mctlcom_get_response(char *response, int size, unsigned int *timeout);

#endif
