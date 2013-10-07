#ifndef STACK_H
#define STACK_H

#define STACK_SIZE 24

typedef struct{
	char data[STACK_SIZE];
	int used;
} stack_t;

#endif
