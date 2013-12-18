#include "motorctl.h"

/***************************
 * Motor control functions *
 ***************************/

inline void motorctl_setup(void)
{
	setup_pid_info();
	setup_trapezoidal_movement();
}
