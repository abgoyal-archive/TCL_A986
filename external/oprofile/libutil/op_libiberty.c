
#include <string.h>

#include "op_libiberty.h"

#ifndef HAVE_XCALLOC
/* some system have a valid libiberty without xcalloc */
void * xcalloc(size_t n_elem, size_t sz)
{
	void * ptr = xmalloc(n_elem * sz);

	memset(ptr, '\0', n_elem * sz);

	return ptr;
}
#endif

#ifndef HAVE_XMEMDUP
void * xmemdup (void const * input, size_t copy_size, size_t alloc_size)
{
	void * output = xcalloc(1, alloc_size);

	memcpy(output, input, copy_size);

	return output;
}
#endif
