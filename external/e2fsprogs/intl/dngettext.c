
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
# define DNGETTEXT __dngettext
# define DCNGETTEXT __dcngettext
#else
# define DNGETTEXT libintl_dngettext
# define DCNGETTEXT libintl_dcngettext
#endif

char *
DNGETTEXT (const char *domainname,
	   const char *msgid1, const char *msgid2, unsigned long int n)
{
  return DCNGETTEXT (domainname, msgid1, msgid2, n, LC_MESSAGES);
}

#ifdef _LIBC
/* Alias for function name in GNU C Library.  */
weak_alias (__dngettext, dngettext);
#endif
