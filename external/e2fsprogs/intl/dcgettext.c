
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "gettextP.h"
#ifdef _LIBC
# include <libintl.h>
#else
# include "libgnuintl.h"
#endif

/* @@ end of prolog @@ */

#ifdef _LIBC
# define DCGETTEXT __dcgettext
# define DCIGETTEXT __dcigettext
#else
# define DCGETTEXT libintl_dcgettext
# define DCIGETTEXT libintl_dcigettext
#endif

char *
DCGETTEXT (const char *domainname, const char *msgid, int category)
{
  return DCIGETTEXT (domainname, msgid, NULL, 0, 0, category);
}

#ifdef _LIBC
/* Alias for function name in GNU C Library.  */
INTDEF(__dcgettext)
weak_alias (__dcgettext, dcgettext);
#endif
