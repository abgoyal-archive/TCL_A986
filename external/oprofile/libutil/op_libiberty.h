
#ifndef OP_LIBIBERTY_H
#define OP_LIBIBERTY_H

#include <stddef.h>

#include "config.h"

#ifdef MALLOC_ATTRIBUTE_OK
#define OP_ATTRIB_MALLOC	__attribute__((malloc))
#else
#define OP_ATTRIB_MALLOC
#endif

#ifdef HAVE_LIBIBERTY_H
#include <libiberty.h>
#else

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_LIBIBERTY_H
/* Set the program name used by xmalloc.  */
void xmalloc_set_program_name(char const *);

void * xmalloc(size_t) OP_ATTRIB_MALLOC;

void * xrealloc(void *, size_t);

/* Allocate memory without fail and set it to zero.  This works like xmalloc */
void * xcalloc(size_t, size_t) OP_ATTRIB_MALLOC;

/* Copy a string into a memory buffer without fail.  */
char * xstrdup(char const *) OP_ATTRIB_MALLOC;

void * xmemdup(void const *, size_t, size_t) OP_ATTRIB_MALLOC;

#endif	/* !HAVE_LIBIBERTY_H */

#ifdef ANDROID
#define xmalloc(s)      malloc(s)
#define xrealloc(p,s)   realloc(p,s)
#define xstrdup(str)    strdup(str)
#define xmalloc_set_program_name(n)
#endif

#ifdef __cplusplus
}
#endif

#endif /* !HAVE_LIBIBERTY_H */

#endif /* OP_LIBIBERTY_H */
