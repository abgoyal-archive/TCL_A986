
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>

#undef __stpcpy
#undef stpcpy

#ifndef weak_alias
# define __stpcpy stpcpy
#endif

/* Copy SRC to DEST, returning the address of the terminating '\0' in DEST.  */
char *
__stpcpy (char *dest, const char *src)
{
  register char *d = dest;
  register const char *s = src;

  do
    *d++ = *s;
  while (*s++ != '\0');

  return d - 1;
}
#ifdef weak_alias
weak_alias (__stpcpy, stpcpy)
#endif
