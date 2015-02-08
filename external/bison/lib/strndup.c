
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif
#if !_LIBC
# include "strndup.h"
#endif

#include <stdlib.h>
#include <string.h>

#if !_LIBC
# include "strnlen.h"
# ifndef __strnlen
#  define __strnlen strnlen
# endif
#endif

#undef __strndup
#if _LIBC
# undef strndup
#endif

#ifndef weak_alias
# define __strndup strndup
#endif

char *
__strndup (s, n)
     const char *s;
     size_t n;
{
  size_t len = __strnlen (s, n);
  char *new = malloc (len + 1);

  if (new == NULL)
    return NULL;

  new[len] = '\0';
  return memcpy (new, s, len);
}
#ifdef libc_hidden_def
libc_hidden_def (__strndup)
#endif
#ifdef weak_alias
weak_alias (__strndup, strndup)
#endif
