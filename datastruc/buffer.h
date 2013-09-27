#ifndef BUFFER_H
#define BUFFER_H

#ifndef _SIZE_T
#define _SIZE_T
typedef unsigned int size_t;
#endif

#ifndef _BYTE_T
#define _BYTE_T
typedef unsigned char byte_t;
#endif

#ifndef _BOOL_T
#define _BOOL_T
typedef unsigned char bool_t;
#define true     1
#define false    0
#endif

#define BUFFER_SIZE 64

/**
 * Type definition for a linear buffer.
 */
typedef struct {
	byte_t    data[BUFFER_SIZE];
	size_t    size;
	size_t    used;
} buffer_t;

/**
 * Initialize the given buffer for being used. Currently, this only consists in
 * setting the buffer's 'size' and 'used' fields to 'BUFFER_SIZE' and zero,
 * respectively.
 */
void buffer_init(buffer_t *buf);

#endif
