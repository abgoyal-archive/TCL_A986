/*	$NetBSD: gcmalloc.h,v 1.4 2006/09/09 16:22:09 manu Exp $	*/

/*	$KAME: gcmalloc.h,v 1.4 2001/11/16 04:34:57 sakane Exp $	*/



#ifndef _GCMALLOC_H_DEFINED
#define _GCMALLOC_H_DEFINED

/* ElectricFence needs no special handling. */

#ifdef GC
#define GC_DEBUG
#include <gc.h>

#ifdef RACOON_MAIN_PROGRAM
void *
malloc(size_t size)
{

	return (GC_MALLOC(size));
}

void *
calloc(size_t number, size_t size)
{

	/* GC_malloc() clears the storage. */
	return (GC_MALLOC(number * size));
}

void *
realloc(void *ptr, size_t size)
{

	return (GC_REALLOC(ptr, size));
}

void
free(void *ptr)
{

	GC_FREE(ptr);
}

char *
strdup(const char *str)
{

	return (GC_STRDUP(str));
}
#endif /* RACOON_MAIN_PROGRAM */

#define	racoon_malloc(sz)	GC_debug_malloc(sz, GC_EXTRAS)
#define	racoon_calloc(cnt, sz)	GC_debug_malloc(cnt * sz, GC_EXTRAS)
#define	racoon_realloc(old, sz)	GC_debug_realloc(old, sz, GC_EXTRAS)
#define	racoon_free(p)		GC_debug_free(p)
#define	racoon_strdup(str)	GC_debug_strdup(str)

#endif /* GC */

#ifdef DMALLOC
#include <dmalloc.h>
#endif /* DMALLOC */

#ifdef DEBUG_RECORD_MALLOCATION
#include <debugrm.h>
#else
#ifndef racoon_malloc
#define	racoon_malloc(sz)	malloc((sz))
#endif
#ifndef racoon_calloc
#define	racoon_calloc(cnt, sz)	calloc((cnt), (sz))
#endif
#ifndef racoon_realloc
#define	racoon_realloc(old, sz)	realloc((old), (sz))
#endif
#ifndef racoon_free
#define	racoon_free(p)		free((p))
#endif
#ifndef racoon_strdup
#define	racoon_strdup(s)	strdup((s))
#endif
#endif /* DEBUG_RECORD_MALLOCATION */

#endif /* _GCMALLOC_H_DEFINED */
