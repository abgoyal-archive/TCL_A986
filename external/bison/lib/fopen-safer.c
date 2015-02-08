
/* Written by Paul Eggert.  */

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "stdio-safer.h"

#include <errno.h>
#include <unistd.h>
#include "unistd-safer.h"

#ifndef STDERR_FILENO
# define STDERR_FILENO 2
#endif

/* Like fopen, but do not return stdin, stdout, or stderr.  */

FILE *
fopen_safer (char const *file, char const *mode)
{
  FILE *fp = fopen (file, mode);

  if (fp)
    {
      int fd = fileno (fp);

      if (0 <= fd && fd <= STDERR_FILENO)
	{
	  int f = dup_safer (fd);

	  if (f < 0)
	    {
	      int e = errno;
	      fclose (fp);
	      errno = e;
	      return NULL;
	    }

	  if (fclose (fp) != 0
	      || ! (fp = fdopen (f, mode)))
	    {
	      int e = errno;
	      close (f);
	      errno = e;
	      return NULL;
	    }
	}
    }

  return fp;
}
