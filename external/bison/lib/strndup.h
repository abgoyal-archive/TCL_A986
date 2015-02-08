
#if HAVE_STRNDUP

/* Get strndup() declaration.  */
#include <string.h>

#else

#include <stddef.h>

/* Return a newly allocated copy of at most N bytes of STRING.  */
extern char *strndup (const char *string, size_t n);

#endif
