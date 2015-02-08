
/* written by Jim Meyering */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif
#undef malloc

#include <stdlib.h>


void *
rpl_malloc (size_t n)
{
  if (n == 0)
    n = 1;
  return malloc (n);
}
