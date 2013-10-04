#include "controller_status.h"

bool_t motor_is_in_trapezoidal_mode(char motor)
{
	return false;
}

bool_t motor_executing_trapezoidal_move(char motor)
{
	return false;
}

bool_t any_motor_executing_trapezoidal_move(char motor_flags)
{
	return false;
}

bool_t hard_home_executed(void)
{
	return false;
}

bool_t controller_in_robot_mode(void)
{
	return false;
}

bool_t hard_home_in_progress(void)
{
	return false;
}

bool_t controller_is_in_teach_pendant_mode(void)
{
	return false;
}

controller_mode_t controller_mode(void)
{
	return GENERIC;
}

bool_t gripper_is_enabled(void)
{
	return false;
}
