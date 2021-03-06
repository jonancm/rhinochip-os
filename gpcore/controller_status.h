#ifndef CONTROLLER_STATUS_H
#define CONTROLLER_STATUS_H

#include "../types.h"
#include "../datastruc/stack.h"

typedef enum {XR3, SCARA, GENERIC} controller_mode_t;

#define BIT_0        0x01
#define BIT_1        0x02
#define BIT_2        0x04
#define BIT_3        0x08
#define BIT_4        0x10
#define BIT_5        0x20
#define BIT_6        0x40
#define BIT_7        0x80

#define MOTOR_A      0x01
#define MOTOR_B      0x02
#define MOTOR_C      0x04
#define MOTOR_D      0x08
#define MOTOR_E      0x10
#define MOTOR_F      0x20
#define MOTOR_G      0x40
#define MOTOR_H      0x80
#define MOTOR_ALL    0xFF

typedef enum {MOTOR_IDLE, MOTOR_TRAPEZOIDAL, MOTOR_VELOCITY, MOTOR_OPEN_LOOP} motor_mode_t;
typedef enum {HARDHOME_NEEDED, HARDHOME_EXECUTED, HARDHOME_IN_PROGRESS} hardhome_status_t;
typedef enum {GRIPPER_OPEN, GRIPPER_CLOSED} gripper_status_t;

typedef struct {
	/**
	 * Motor status (whether the motor is executing a trapezoidal move or not).
	 * 
	 * Bit 7:    1 = Motor H in Trapezoidal move    0 = Motor H not in Trapezoidal move
	 * Bit 6:    1 = Motor G in Trapezoidal move    0 = Motor G not in Trapezoidal move
	 * Bit 5:    1 = Motor F in Trapezoidal move    0 = Motor F not in Trapezoidal move
	 * Bit 4:    1 = Motor E in Trapezoidal move    0 = Motor E not in Trapezoidal move
	 * Bit 3:    1 = Motor D in Trapezoidal move    0 = Motor D not in Trapezoidal move
	 * Bit 2:    1 = Motor C in Trapezoidal move    0 = Motor C not in Trapezoidal move
	 * Bit 1:    1 = Motor B in Trapezoidal move    0 = Motor B not in Trapezoidal move
	 * Bit 0:    1 = Motor A in Trapezoidal move    0 = Motor A not in Trapezoidal move
	 */
	unsigned char motor_status; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	/**
	 * System Configuration.
	 * 
	 * Bit 7:    1 = System is in host mode     0 = System is in pendant mode    (System Mode)
	 * Bit 6:    1 = The pendant is enabled     0 = The pendant is disabled      (Pendant Configuration)
	 * Bit 5:    1 = Generic controller mode    0 = Robot controller mode        (Controller Mode)
	 * Bit 4:    1 = SCARA mode                 0 = XR-3 mode                    (Robot Type)
	 * Bit 3:    1 = The gripper is disabled    0 = The gripper is enabled       (Gripper Configuration)
	 * Bit 2:    1 = XYZ mode                   0 = Joint mode                   (Coordinate Mode)
	 * Bit 1:    always 0
	 * Bit 0:    always 0
	 */
	unsigned char system_config;
	
	/**
	 * System Status.
	 * 
	 * Bit 7: 1 = At least one motor is performing a trapezoidal move.
	 * Bit 6: 1 = A system error has occurred.
	 * Bit 5: 1 = The general purpose delay timer is active.
	 * Bit 4: 1 = At least one wait on input or wait on switch is still pending.
	 * Bit 3: 1 = No teach pendant is connected.
	 * Bit 2: 1 = The teach pendant ENTER key has been pressed.
	 * Bit 1: 1 = The teach pendant ESCAPE key has been pressed.
	 * Bit 0: 1 = A teach pendant error has occurred.
	 */
	unsigned char system_status;
	
	/**
	 * Delay Timer Value.
	 * 
	 * 0 <= d <= 3000
	 * Units of 1/10 second
	 */
	int delay_timer;
	
	/**
	 * Host Error Stack.
	 * 
	 * 24-elements deep LIFO queue.
	 */
	stack_t error_stack;
	
	/**
	 * Motor Mode.
	 * 
	 * 0 = Idle Mode           (MOTOR_IDLE)
	 * 1 = Trapezoidal mode    (MOTOR_TRAPEZOIDAL)
	 * 2 = Velocity mode       (MOTOR_VELOCITY)
	 * 3 = Open Loop mode      (MOTOR_OPEN_LOOP)
	 */
	struct {
		motor_mode_t    motor_a;
		motor_mode_t    motor_b;
		motor_mode_t    motor_c;
		motor_mode_t    motor_d;
		motor_mode_t    motor_e;
		motor_mode_t    motor_f;
		motor_mode_t    motor_g;
		motor_mode_t    motor_h;
	} motor_mode; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	unsigned long int usage_time;
	
	struct {
		int motor_a;
		int motor_b;
		int motor_c;
		int motor_d;
		int motor_e;
		int motor_f;
		int motor_g;
		int motor_h;
	} current_position; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	struct {
		int motor_a;
		int motor_b;
		int motor_c;
		int motor_d;
		int motor_e;
		int motor_f;
		int motor_g;
		int motor_h;
	} relative_destination; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	struct {
		int motor_a;
		int motor_b;
		int motor_c;
		int motor_d;
		int motor_e;
		int motor_f;
		int motor_g;
		int motor_h;
	} absolute_destination; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	struct {
		float x;
		float y;
		float z;
		float a;
		float t;
	} xyz_destination; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	char system_acceleration; // TODO: remove? Is this needed? It is stored in the MCMCU!
	
	gripper_status_t gripper_status; // TODO: remove? Is this needed? It should be stored in the MCMCU!
	
	struct {
		int motor_a;
		int motor_b;
		int motor_c;
		int motor_d;
		int motor_e;
		int motor_f;
		int motor_g;
		int motor_h;
	} soft_home_position;
	
	unsigned char limit_switches;
	
	struct {
		char motor_a;
		char motor_b;
		char motor_c;
		char motor_d;
		char motor_e;
		char motor_f;
		char motor_g;
		char motor_h;
	} motor_actual_velocity; // TODO: remove? Is this needed? It should be stored in the MCMCU!
	
	char input_ports;
	
	char input_switches;
	
	/**
	 * Flag to indicate whether a Hard Home has been executed or not.
	 */
	hardhome_status_t hardhome_status;
} controller_status_t;

#ifndef CONTROLLER_STATUS_C
extern controller_status_t    controller;
#endif

void controller_status_setup(void);

bool_t motor_is_in_trapezoidal_mode(unsigned char motor);
void set_motor_mode(unsigned char motor_flags, motor_mode_t motor_mode);

void update_motor_status(void);
void update_system_status(void);
void update_motor_mode(void);

/**
 * Check if any of the given motors is executing a trapezoidal move.
 * 
 * If any of the given motors is in trapezoidal mode and executing a trapezoidal move, return true.
 * If any of the given motors is not in trapezoidal mode or is not executing a trapezoidal move,
 * return false.
 */
bool_t any_motor_executing_trapezoidal_move(unsigned char motor_flags);

/**
 * Check if a hard home has been executed since system startup.
 */
bool_t hard_home_executed(void);

/**
 * Check if the controller is in robot mode (i.e. XR-3 or SCARA mode).
 */
bool_t controller_in_robot_mode(void);

/**
 * Check if a hard home is in progress.
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
inline void disable_gripper(void);
inline void enable_gripper(void);

void set_controller_generic_mode(void);
void set_controller_xr3_mode(void);
void set_controller_scara_mode(void);

void update_system_acceleration(void);

void reset_soft_home(void);

void update_limit_switches(void);

#endif
