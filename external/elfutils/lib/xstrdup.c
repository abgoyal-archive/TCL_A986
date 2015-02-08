
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string.h>
#include "system.h"


/* Return a newly allocated copy of STRING.  */
char *
xstrdup (string)
     const char *string;
{
  return strcpy (xmalloc (strlen (string) + 1), string);
}
