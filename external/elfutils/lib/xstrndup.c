
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>
#include "system.h"


/* Return a newly allocated copy of STRING.  */
char *
xstrndup (string, n)
     const char *string;
     size_t n;
{
  char *res;
  size_t len = strnlen (string, n);
  *((char *) mempcpy ((res = xmalloc (len + 1)), string, len)) = '\0';
  return res;
}
