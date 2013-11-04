#ifndef MOTOR_STATUS_C
#define MOTOR_STATUS_C
#endif

#include "motor_status.h"

/**
 * 16-bit count registers to count motor steps. Being 16-bit wide, they can
 * store a value between -32768 an 32767.
 */
int motor_steps[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};

/**
 * Flag that indicates whether a motor is stalled.
 */
bool_t motor_stalled[NUM_MOTORS] = {false, false, false, false, false, false};
