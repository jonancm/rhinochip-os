#ifndef CONTROLLER_STATUS_H
#define CONTROLLER_STATUS_H

#include "../types.h"

typedef enum {XR3, SCARA, GENERIC} controller_mode_t;

#define MOTOR_A      0x01
#define MOTOR_B      0x02
#define MOTOR_C      0x04
#define MOTOR_D      0x08
#define MOTOR_E      0x10
#define MOTOR_F      0x20
#define MOTOR_G      0x40
#define MOTOR_H      0x80
#define MOTOR_ALL    0xFF

bool_t motor_is_in_trapezoidal_mode(char motor);

/**
 * Check if the given motors are executing a trapezoidal move.
 * 
 * If the given motors are in trapezoidal mode and executing a trapezoidal move, return true.
 * If the given motors are not in trapezoidal mode or are not executing a trapezoidal move,
 * return false.
 */
bool_t motor_executing_trapezoidal_move(char motor);

bool_t any_motor_executing_trapezoidal_move(char motor_flags);

/**
 * Check if a hard home has been executed since system startup.
 */
bool_t hard_home_executed(void);

/**
 * Check if the controller is in robot mode (i.e. XR-3 or SCARA mode).
 */
bool_t controller_in_robot_mode(void);

/**
 * Check if a hard home is in progres..
 */
bool_t hard_home_in_progress(void);

/**
 * Check if the controller is in teach pendant mode.
 */
bool_t controller_is_in_teach_pendant_mode(void);

/**
 * Return the controller mode (either GENERIC, XR-3 or SCARA).
 */
controller_mode_t controller_mode(void);

bool_t gripper_is_enabled(void);

#endif
