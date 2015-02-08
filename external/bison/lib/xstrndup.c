
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

/* Specification.  */
#include "xstrndup.h"

#include "strndup.h"
#include "xalloc.h"

char *
xstrndup (const char *string, size_t n)
{
  char *s = strndup (string, n);
  if (! s)
    xalloc_die ();
  return s;
}
