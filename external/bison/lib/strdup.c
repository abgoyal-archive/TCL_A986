
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#ifndef _LIBC
/* Get specification.  */
# include "strdup.h"
#endif

#include <stdlib.h>
#include <string.h>

#undef __strdup
#undef strdup

#ifndef weak_alias
# define __strdup strdup
#endif

/* Duplicate S, returning an identical malloc'd string.  */
char *
__strdup (const char *s)
{
  size_t len = strlen (s) + 1;
  void *new = malloc (len);

  if (new == NULL)
    return NULL;

  return (char *) memcpy (new, s, len);
}
#ifdef libc_hidden_def
libc_hidden_def (__strdup)
#endif
#ifdef weak_alias
weak_alias (__strdup, strdup)
#endif
