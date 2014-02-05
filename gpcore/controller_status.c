#ifndef CONTROLLER_STATUS_C
#define CONTROLLER_STATUS_C
#endif

#include "controller_status.h"
#include "mctlcom.h"

#include <stdlib.h> // atoi

controller_status_t    controller;

void controller_status_setup(void)
{
	// At startup, the system configuration is read from the EEPROM.
	// TODO: implement this
	
	// The RhinoChip platform doesn't allow a teach pendant to be connect.
	// Thus, bit 3 of the system status register must be set.
	// At system startup, the other bits are cleared.
	controller.system_status = BIT_3;
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
	buf[recvd] = '\0';
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_A;
	else
		controller.motor_status &= ~MOTOR_A;

	mcuicom_send("XB" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_B;
	else
		controller.motor_status &= ~MOTOR_B;

	mcuicom_send("XC" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_C;
	else
		controller.motor_status &= ~MOTOR_C;

	mcuicom_send("XD" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_D;
	else
		controller.motor_status &= ~MOTOR_D;

	mcuicom_send("XE" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
	status_bit = atoi(buf);
	if (status_bit)
		controller.motor_status |= MOTOR_E;
	else
		controller.motor_status &= ~MOTOR_E;

	mcuicom_send("XF" CMDEND);
	recvd = mctlcom_get_response(buf, size);
	buf[recvd] = '\0';
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
