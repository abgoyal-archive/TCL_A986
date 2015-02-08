
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "gettextP.h"

#include <locale.h>

#ifdef _LIBC
# include <libintl.h>
#else
# include "libgnuintl.h"
#endif

/* @@ end of prolog @@ */

#ifdef _LIBC
# define DGETTEXT __dgettext
# define DCGETTEXT INTUSE(__dcgettext)
#else
# define DGETTEXT libintl_dgettext
# define DCGETTEXT libintl_dcgettext
#endif

char *
DGETTEXT (const char *domainname, const char *msgid)
{
  return DCGETTEXT (domainname, msgid, LC_MESSAGES);
}

#ifdef _LIBC
/* Alias for function name in GNU C Library.  */
weak_alias (__dgettext, dgettext);
#endif
