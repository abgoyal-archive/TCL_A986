
#ifndef	_MALLOC_INTERNAL
#define	_MALLOC_INTERNAL
#include "./mtrace.h"
#endif

#include <stdio.h>

#ifndef	__GNU_LIBRARY__
extern char *getenv ();
#else
#include <stdlib.h>
#endif

static FILE *mallstream;
static char mallenv[]= "MALLOC_TRACE";
static char mallbuf[BUFSIZ];	/* Buffer for the output.  */

/* Address to breakpoint on accesses to... */
__ptr_t mallwatch;

/* Old hook values.  */
static void (*tr_old_free_hook) __P ((__ptr_t ptr));
static __ptr_t (*tr_old_malloc_hook) __P ((size_t size));
static __ptr_t (*tr_old_realloc_hook) __P ((__ptr_t ptr, size_t size));

FILE *malloc_get_mallstream()
{
	return mallstream;
}


void tr_break __P ((void));
void
tr_break ()
{
}

static void tr_freehook __P ((__ptr_t));
static void
tr_freehook (ptr)
     __ptr_t ptr;
{
  fprintf (mallstream, "- %p\n", ptr);	/* Be sure to print it first.  */
  if (ptr == mallwatch)
    tr_break ();
  __free_hook = tr_old_free_hook;
  free (ptr);
  __free_hook = tr_freehook;
}

static __ptr_t tr_mallochook __P ((size_t));
static __ptr_t
tr_mallochook (size)
     size_t size;
{
  __ptr_t hdr;

  __malloc_hook = tr_old_malloc_hook;
  hdr = (__ptr_t) malloc (size);
  __malloc_hook = tr_mallochook;

  /* We could be printing a NULL here; that's OK.  */
  fprintf (mallstream, "+ %p %d\n", hdr, size);

  if (hdr == mallwatch)
    tr_break ();

  return hdr;
}

static __ptr_t tr_reallochook __P ((__ptr_t, size_t));
static __ptr_t
tr_reallochook (ptr, size)
     __ptr_t ptr;
     size_t size;
{
  __ptr_t hdr;

  if (ptr == mallwatch)
    tr_break ();

  __free_hook = tr_old_free_hook;
  __malloc_hook = tr_old_malloc_hook;
  __realloc_hook = tr_old_realloc_hook;
  hdr = (__ptr_t) realloc (ptr, size);
  __free_hook = tr_freehook;
  __malloc_hook = tr_mallochook;
  __realloc_hook = tr_reallochook;
  if (hdr == NULL)
    /* Failed realloc.  */
    fprintf (mallstream, "! %p %d\n", ptr, size);
  else
    fprintf (mallstream, "< %p\n> %p %d\n", ptr, hdr, size);

  if (hdr == mallwatch)
    tr_break ();

  return hdr;
}


void
mtrace ()
{
  char *mallfile;

  mallfile = getenv (mallenv);
  if (mallfile != NULL || mallwatch != NULL)
    {
      mallstream = fopen (mallfile != NULL ? mallfile : "/dev/null", "w");
      if (mallstream != NULL)
	{
	  /* Be sure it doesn't malloc its buffer!  */
	  setbuf (mallstream, mallbuf);
	  fprintf (mallstream, "= Start\n");
	  tr_old_free_hook = __free_hook;
	  __free_hook = tr_freehook;
	  tr_old_malloc_hook = __malloc_hook;
	  __malloc_hook = tr_mallochook;
	  tr_old_realloc_hook = __realloc_hook;
	  __realloc_hook = tr_reallochook;
	}
    }
}
