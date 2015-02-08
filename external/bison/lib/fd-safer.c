
/* Written by Paul Eggert.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "unistd-safer.h"

#include <errno.h>

#include <unistd.h>
#ifndef STDIN_FILENO
# define STDIN_FILENO 0
#endif
#ifndef STDERR_FILENO
# define STDERR_FILENO 2
#endif


int
fd_safer (int fd)
{
  if (STDIN_FILENO <= fd && fd <= STDERR_FILENO)
    {
      int f = dup_safer (fd);
      int e = errno;
      close (fd);
      errno = e;
      fd = f;
    }

  return fd;
}
