#ifndef CONTROLLER_STATUS_C
#define CONTROLLER_STATUS_C
#endif

#include "controller_status.h"

controller_status_t    controller;

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

bool_t any_motor_executing_trapezoidal_move(unsigned char motor_flags)
{
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
