
#ifndef STRDUP_H_
#define STRDUP_H_

/* Get strdup declaration, if available.  */
#include <string.h>

#if defined HAVE_DECL_STRDUP && !HAVE_DECL_STRDUP && !defined strdup
/* Duplicate S, returning an identical malloc'd string.  */
extern char *strdup (const char *s);
#endif

#endif /* STRDUP_H_ */
