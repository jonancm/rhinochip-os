#ifndef CONTROLLER_STATUS_H
#define CONTROLLER_STATUS_H

typedef struct {
	char system_velocity;
} controller_status_t;

#ifndef CONTROLLER_STATUS_C
extern controller_status_t    controller;
#endif

void controller_status_setup(void);

#endif
