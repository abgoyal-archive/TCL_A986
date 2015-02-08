
#ifndef STRNLEN_H
#define STRNLEN_H

/* Get strnlen declaration, if available.  */
#include <string.h>

#if defined HAVE_DECL_STRNLEN && !HAVE_DECL_STRNLEN
extern size_t strnlen(const char *string, size_t maxlen);
#endif

#endif /* STRNLEN_H */
