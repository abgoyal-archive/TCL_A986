
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "xalloc.h"

#include <stdlib.h>
#include <string.h>

#ifndef SIZE_MAX
# define SIZE_MAX ((size_t) -1)
#endif

#if defined HAVE_CALLOC || defined __GLIBC__
enum { HAVE_GNU_CALLOC = 1 };
#else
enum { HAVE_GNU_CALLOC = 0 };
#endif


static inline void *
xnmalloc_inline (size_t n, size_t s)
{
  void *p;
  if (xalloc_oversized (n, s) || (! (p = malloc (n * s)) && n != 0))
    xalloc_die ();
  return p;
}

void *
xnmalloc (size_t n, size_t s)
{
  return xnmalloc_inline (n, s);
}

/* Allocate N bytes of memory dynamically, with error checking.  */

void *
xmalloc (size_t n)
{
  return xnmalloc_inline (n, 1);
}


static inline void *
xnrealloc_inline (void *p, size_t n, size_t s)
{
  if (xalloc_oversized (n, s) || (! (p = realloc (p, n * s)) && n != 0))
    xalloc_die ();
  return p;
}

void *
xnrealloc (void *p, size_t n, size_t s)
{
  return xnrealloc_inline (p, n, s);
}


void *
xrealloc (void *p, size_t n)
{
  return xnrealloc_inline (p, n, 1);
}



static inline void *
x2nrealloc_inline (void *p, size_t *pn, size_t s)
{
  size_t n = *pn;

  if (! p)
    {
      if (! n)
	{
	  /* The approximate size to use for initial small allocation
	     requests, when the invoking code specifies an old size of
	     zero.  64 bytes is the largest "small" request for the
	     GNU C library malloc.  */
	  enum { DEFAULT_MXFAST = 64 };

	  n = DEFAULT_MXFAST / s;
	  n += !n;
	}
    }
  else
    {
      if (SIZE_MAX / 2 / s < n)
	xalloc_die ();
      n *= 2;
    }

  *pn = n;
  return xrealloc (p, n * s);
}

void *
x2nrealloc (void *p, size_t *pn, size_t s)
{
  return x2nrealloc_inline (p, pn, s);
}


void *
x2realloc (void *p, size_t *pn)
{
  return x2nrealloc_inline (p, pn, 1);
}


void *
xzalloc (size_t s)
{
  return memset (xmalloc (s), 0, s);
}


void *
xcalloc (size_t n, size_t s)
{
  void *p;
  /* Test for overflow, since some calloc implementations don't have
     proper overflow checks.  But omit overflow and size-zero tests if
     HAVE_GNU_CALLOC, since GNU calloc catches overflow and never
     returns NULL if successful.  */
  if ((! HAVE_GNU_CALLOC && xalloc_oversized (n, s))
      || (! (p = calloc (n, s)) && (HAVE_GNU_CALLOC || n != 0)))
    xalloc_die ();
  return p;
}


void *
xmemdup (void const *p, size_t s)
{
  return memcpy (xmalloc (s), p, s);
}

/* Clone STRING.  */

char *
xstrdup (char const *string)
{
  return xmemdup (string, strlen (string) + 1);
}
