
/* Written by Paul Eggert.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <errno.h>

#include "get-errno.h"


int
get_errno (void)
{
  return errno;
}

void
set_errno (int e)
{
  errno = e;
}
