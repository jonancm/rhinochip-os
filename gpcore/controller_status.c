#ifndef CONTROLLER_STATUS_C
#define CONTROLLER_STATUS_C
#endif

#include "controller_status.h"
#include "mctlcom.h"
#include "hardhome.h"

#include <stdlib.h> // atoi
#include <stdio.h>  // snprintf

controller_status_t    controller;

void set_motor_mode(unsigned char motor_flags, motor_mode_t motor_mode)
{
	const int size = 64;
	char buf[size];
	
	if (motor_flags & MOTOR_A)
	{
		snprintf(buf, size, "NA,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_a = motor_mode;
	}
	
	if (motor_flags & MOTOR_B)
	{
		snprintf(buf, size, "NB,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_b = motor_mode;
	}
	
	if (motor_flags & MOTOR_C)
	{
		snprintf(buf, size, "NC,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_c = motor_mode;
	}

	if (motor_flags & MOTOR_D)
	{
		snprintf(buf, size, "ND,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_d = motor_mode;
	}
	
	if (motor_flags & MOTOR_E)
	{
		snprintf(buf, size, "NE,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_e = motor_mode;
	}
	
	if (motor_flags & MOTOR_F)
	{
		snprintf(buf, size, "NF,%d" CMDEND, motor_mode);
		mcuicom_send(buf);
		controller.motor_mode.motor_f = motor_mode;
	}
}

void setup_pid_gains(void)
{
	// TODO: Set PID gains to the values stored in the EEPROM

	// Set PID gains for motor A
	// mcuicom_send("UP,A,46" CMDEND);
	// mcuicom_send("UI,A,40" CMDEND);
	// mcuicom_send("UD,A,110" CMDEND);
	mcuicom_send("UP,A,1" CMDEND);
	mcuicom_send("UI,A,0" CMDEND);
	mcuicom_send("UD,A,0" CMDEND);
}

void controller_status_setup(void)
{
	// At startup, the system configuration is read from the EEPROM.
	// TODO: implement this
	
	// System configuration
	// 
	// Bit 7:    0 = System is in pendant mode    (System Mode)
	// Bit 6:    1 = The pendant is enabled       (Pendant Configuration)
	// Bit 5:    0 = Robot controller mode        (Controller Mode)
	// Bit 4:    0 = XR-3 mode                    (Robot Type)
	// Bit 3:    0 = The gripper is enabled       (Gripper Configuration)
	// Bit 2:    0 = Joint mode                   (Coordinate Mode)
	// Bit 1:    always 0
	// Bit 0:    always 0
	controller.system_config = 0b01000000;

	// The RhinoChip platform doesn't allow a teach pendant to be connected.
	// Thus, bit 3 of the system status register must be set.
	// At system startup, the other bits are cleared.
	// 
	// Bit 7: 0 = No motor is performing a trapezoidal move.
	// Bit 6: 0 = No system error has occurred.
	// Bit 5: 0 = The general purpose delay timer is inactive.
	// Bit 4: 0 = No wait on input or wait on switch is still pending.
	// Bit 3: 1 = No teach pendant is connected.
	// Bit 2: 0 = The teach pendant ENTER key has not been pressed.
	// Bit 1: 0 = The teach pendant ESCAPE key has not been pressed.
	// Bit 0: 0 = No teach pendant error has occurred.
	controller.system_status = BIT_3;

	// Set up motor mode on startup
	// TODO: read settings from EEPROM rather than hardcoding them into the program
	set_motor_mode(MOTOR_ALL, MOTOR_IDLE);
	set_motor_mode(MOTOR_A, MOTOR_TRAPEZOIDAL);

	// Indicate that a hard home has not been executed
	controller.hardhome_status = HARDHOME_NEEDED;

	// Reset soft home position (all motors to zero)
	reset_soft_home();

	// Set up PID gains
	setup_pid_gains();
}

bool_t motor_is_in_trapezoidal_mode(unsigned char motor)
{
	bool_t condition = false;
	
	switch (motor)
	{
		case MOTOR_A:
			condition = controller.motor_mode.motor_a == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_B:
			condition = controller.motor_mode.motor_b == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_C:
			condition = controller.motor_mode.motor_c == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_D:
			condition = controller.motor_mode.motor_d == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_E:
			condition = controller.motor_mode.motor_e == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_F:
			condition = controller.motor_mode.motor_f == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_G:
			condition = controller.motor_mode.motor_g == MOTOR_TRAPEZOIDAL;
			break;
		case MOTOR_H:
			condition = controller.motor_mode.motor_h == MOTOR_TRAPEZOIDAL;
			break;
	}
	
	return condition;
}

void update_motor_status(void)
{
	const int size = 64;
	char buf[size];
	int recvd, status_bit;
	
	mcuicom_send("XA" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_A;
	else
		controller.motor_status &= ~MOTOR_A;

	mcuicom_send("XB" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_B;
	else
		controller.motor_status &= ~MOTOR_B;

	mcuicom_send("XC" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_C;
	else
		controller.motor_status &= ~MOTOR_C;

	mcuicom_send("XD" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_D;
	else
		controller.motor_status &= ~MOTOR_D;

	mcuicom_send("XE" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_E;
	else
		controller.motor_status &= ~MOTOR_E;

	mcuicom_send("XF" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_F;
	else
		controller.motor_status &= ~MOTOR_F;
}

bool_t any_motor_executing_trapezoidal_move(unsigned char motor_flags)
{
	update_motor_status();
	return controller.motor_status & motor_flags;
}

bool_t hard_home_executed(void)
{
	return controller.hardhome_status == HARDHOME_EXECUTED;
}

bool_t controller_in_robot_mode(void)
{
	return !(controller.system_config & BIT_5);
}

bool_t hard_home_in_progress(void)
{
	return controller.hardhome_status == HARDHOME_IN_PROGRESS;
}

bool_t controller_is_in_teach_pendant_mode(void)
{
	return !(controller.system_config & BIT_7);
}

controller_mode_t controller_mode(void)
{
	controller_mode_t mode;
	
	if (controller.system_config & BIT_5)
		mode = GENERIC;
	else
	{
		if (controller.system_config & BIT_4)
			mode = SCARA;
		else
			mode = XR3;
	}
	
	return mode;
}

bool_t gripper_is_enabled(void)
{
	return !(controller.system_config & BIT_3);
}

inline void disable_gripper(void)
{
	// Set bit 3: disable gripper
	controller.system_config |= BIT_3;
}

inline void enable_gripper(void)
{
	// Clear bit 3: enable gripper
	controller.system_config &= ~BIT_3;
	// TODO: close then open gripper (for compatibility with the Mark IV)
}

void update_system_status(void)
{
	bool_t flag = any_motor_executing_trapezoidal_move(MOTOR_ALL);
	if (flag)
		controller.system_status |= BIT_7;
	else
		controller.system_status &= ~BIT_7;
}

void set_controller_generic_mode(void)
{
	// Set bit 5: set controller mode to generic mode
	controller.system_config |= BIT_5;
	// Automatically disable the gripper
	disable_gripper();
	// TODO: reset PID gains (needs EEPROM support)
}

void set_controller_xr3_mode(void)
{
	// Clear bit 5: set controller mode to robot mode
	controller.system_config &= ~BIT_5;
	// Clear bit 5: set robot type to XR-3
	controller.system_config &= ~BIT_4;
	// Automatically enable the gripper
	enable_gripper();
	// TODO: reset PID gains (needs EEPROM support)
}

void set_controller_scara_mode(void)
{
	// Clear bit 5: set controller mode to robot mode
	controller.system_config &= ~BIT_5;
	// Set bit 5: set robot type to SCARA
	controller.system_config |= BIT_4;
	// Automatically enable the gripper
	enable_gripper();
	// TODO: reset PID gains (needs EEPROM support)
}

void update_motor_mode(void)
{
	const int size = 64;
	char buf[size];
	int recvd;
	motor_mode_t motor_mode;
	
	mcuicom_send("QA" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_a = motor_mode;

	mcuicom_send("QB" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_b = motor_mode;

	mcuicom_send("QC" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_c = motor_mode;

	mcuicom_send("QD" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_d = motor_mode;

	mcuicom_send("QE" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_e = motor_mode;

	mcuicom_send("QF" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0'; // 'atoi' expects a C-string, therefore, must add null terminator
	motor_mode = atoi(buf);
	controller.motor_mode.motor_f = motor_mode;
}

void update_system_acceleration(void)
{
	const int size = 64;
	char buf[size];
	int recvd;

	mcuicom_send("AR" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	// TODO: the previous line may be moved to 'mcuicom_read_cmd'.
	// Another possible solution may be to enable 'hostcom_send' to
	// accept a 'size' parameter that tells the size of the buffer.
	controller.system_acceleration = atoi(buf);
}

void reset_soft_home(void)
{
	controller.soft_home_position.motor_a = 0;
	controller.soft_home_position.motor_b = 0;
	controller.soft_home_position.motor_c = 0;
	controller.soft_home_position.motor_d = 0;
	controller.soft_home_position.motor_e = 0;
	controller.soft_home_position.motor_f = 0;
	controller.soft_home_position.motor_g = 0;
	controller.soft_home_position.motor_h = 0;
}

void update_limit_switches(void)
{
	// In the limit switch status register, a value of 0 indicates that the
	// limit switch is closed (on or active), and a value of 1 indicates that
	// the limit switch is open (off or inactive).
	// 
	// However, the readings from the hardware are inverted. Thus, when the
	// corresponding input port is 0, the switch is open or inactive, and, when
	// the input port is 1, the switch is closed or active.
	controller.limit_switches  = 0xFF;
	controller.limit_switches &= ~(LMT_MF << 5);
	controller.limit_switches &= ~(LMT_ME << 4);
	controller.limit_switches &= ~(LMT_MD << 3);
	controller.limit_switches &= ~(LMT_MC << 2);
	controller.limit_switches &= ~(LMT_MB << 1);
	controller.limit_switches &= ~(LMT_MA);
}
