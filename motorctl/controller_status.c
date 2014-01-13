#ifndef CONTROLLER_STATUS_C
#define CONTROLLER_STATUS_C
#endif

#include "controller_status.h"

controller_status_t    controller;

void controller_status_setup(void)
{
	controller.system_velocity = 100;
}
