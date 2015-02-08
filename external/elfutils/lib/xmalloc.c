
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <error.h>
//#include <libintl.h>
#include <stddef.h>
#include <stdlib.h>
#include <sys/types.h>
#include "system.h"

#ifndef _
# define _(str) gettext (str)
#endif


/* Allocate N bytes of memory dynamically, with error checking.  */
void *
xmalloc (n)
     size_t n;
{
  void *p;

  p = malloc (n);
  if (p == NULL)
    error (EXIT_FAILURE, 0, _("memory exhausted"));
  return p;
}


/* Allocate memory for N elements of S bytes, with error checking.  */
void *
xcalloc (n, s)
     size_t n, s;
{
  void *p;

  p = calloc (n, s);
  if (p == NULL)
    error (EXIT_FAILURE, 0, _("memory exhausted"));
  return p;
}


void *
xrealloc (p, n)
     void *p;
     size_t n;
{
  p = realloc (p, n);
  if (p == NULL)
    error (EXIT_FAILURE, 0, _("memory exhausted"));
  return p;
}
