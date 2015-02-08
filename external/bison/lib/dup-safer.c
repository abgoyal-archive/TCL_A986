
/* Written by Paul Eggert.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "unistd-safer.h"

#include <fcntl.h>

#include <unistd.h>
#ifndef STDERR_FILENO
# define STDERR_FILENO 2
#endif


int
dup_safer (int fd)
{
#ifdef F_DUPFD
  return fcntl (fd, F_DUPFD, STDERR_FILENO + 1);
#else
  /* fd_safer calls us back, but eventually the recursion unwinds and
     does the right thing.  */
  return fd_safer (dup (fd));
#endif
}
