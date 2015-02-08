
/* Written by Jim Meyering.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "unistd-safer.h"

#include <unistd.h>


int
pipe_safer (int fd[2])
{
  int fail = pipe (fd);
  if (fail)
    return fail;

  {
    int i;
    for (i = 0; i < 2; i++)
      {
	int f = fd_safer (fd[i]);
	if (f < 0)
	  return -1;
	fd[i] = f;
      }
  }

  return 0;
}
