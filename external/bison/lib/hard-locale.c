
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "hard-locale.h"

#include <locale.h>
#include <stdlib.h>
#include <string.h>

#include "strdup.h"

#ifdef __GLIBC__
# define GLIBC_VERSION __GLIBC__
#else
# define GLIBC_VERSION 0
#endif

bool
hard_locale (int category)
{
  bool hard = true;
  char const *p = setlocale (category, NULL);

  if (p)
    {
      if (2 <= GLIBC_VERSION)
	{
	  if (strcmp (p, "C") == 0 || strcmp (p, "POSIX") == 0)
	    hard = false;
	}
      else
	{
	  char *locale = strdup (p);
	  if (locale)
	    {
	      /* Temporarily set the locale to the "C" and "POSIX" locales
		 to find their names, so that we can determine whether one
		 or the other is the caller's locale.  */
	      if (((p = setlocale (category, "C"))
		   && strcmp (p, locale) == 0)
		  || ((p = setlocale (category, "POSIX"))
		      && strcmp (p, locale) == 0))
		hard = false;

	      /* Restore the caller's locale.  */
	      setlocale (category, locale);
	      free (locale);
	    }
	}
    }

  return hard;
}
