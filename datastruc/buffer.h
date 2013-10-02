#ifndef BUFFER_H
#define BUFFER_H

#include "../types.h"

#define BUFFER_SIZE 64

/**
 * Type definition for a linear buffer.
 */
typedef struct {
	char    data[BUFFER_SIZE];
	int     size;
	int     used;
} buffer_t;

/**
 * Initialize the given buffer for being used. Currently, this only consists in
 * setting the buffer's 'size' and 'used' fields to 'BUFFER_SIZE' and zero,
 * respectively.
 */
void buffer_init(buffer_t *buf);

#endif
