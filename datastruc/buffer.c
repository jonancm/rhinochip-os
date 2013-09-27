#include "buffer.h"

#include <stdlib.h>

void buffer_init(buffer_t *buf)
{
	if (buf != NULL)
	{
		buf->size = BUFFER_SIZE;
		buf->used = 0;
	}
}
