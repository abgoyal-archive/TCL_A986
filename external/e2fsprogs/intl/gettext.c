
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#ifdef _LIBC
# define __need_NULL
# include <stddef.h>
#else
# include <stdlib.h>		/* Just for NULL.  */
#endif

#include "gettextP.h"
#ifdef _LIBC
# include <libintl.h>
#else
# include "libgnuintl.h"
#endif

/* @@ end of prolog @@ */

#ifdef _LIBC
# define GETTEXT __gettext
# define DCGETTEXT INTUSE(__dcgettext)
#else
# define GETTEXT libintl_gettext
# define DCGETTEXT libintl_dcgettext
#endif

char *
GETTEXT (const char *msgid)
{
  return DCGETTEXT (NULL, msgid, LC_MESSAGES);
}

#ifdef _LIBC
/* Alias for function name in GNU C Library.  */
weak_alias (__gettext, gettext);
#endif
