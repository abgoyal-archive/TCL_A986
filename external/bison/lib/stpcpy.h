
#ifndef _STPCPY_H
#define _STPCPY_H

#if HAVE_STPCPY

/* Get stpcpy() declaration.  */
#include <string.h>

#else

#ifdef __cplusplus
extern "C" {
#endif

/* Copy SRC to DST, returning the address of the terminating '\0' in DST.  */
extern char *stpcpy (char *dst, const char *src);

#ifdef __cplusplus
}
#endif

#endif

#endif /* _STPCPY_H */
