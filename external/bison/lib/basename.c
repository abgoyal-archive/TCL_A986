
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "dirname.h"
#include <string.h>


char *
base_name (char const *name)
{
  char const *base = name + FILE_SYSTEM_PREFIX_LEN (name);
  char const *p;

  for (p = base; *p; p++)
    {
      if (ISSLASH (*p))
	{
	  /* Treat multiple adjacent slashes like a single slash.  */
	  do p++;
	  while (ISSLASH (*p));

	  /* If the file name ends in slash, use the trailing slash as
	     the basename if no non-slashes have been found.  */
	  if (! *p)
	    {
	      if (ISSLASH (*base))
		base = p - 1;
	      break;
	    }

	  /* *P is a non-slash preceded by a slash.  */
	  base = p;
	}
    }

  return (char *) base;
}


size_t
base_len (char const *name)
{
  size_t len;

  for (len = strlen (name);  1 < len && ISSLASH (name[len - 1]);  len--)
    continue;

  return len;
}
