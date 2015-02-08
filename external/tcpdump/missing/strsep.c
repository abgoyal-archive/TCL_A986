

#if defined(LIBC_SCCS) && !defined(lint)
static const char rcsid[] =
    "@(#) $Header: /tcpdump/master/tcpdump/missing/strsep.c,v 1.3 2003/03/25 08:33:48 guy Exp $ (LBL)";
#endif /* LIBC_SCCS and not lint */

#ifdef HAVE_CONFIG_H 
#include <config.h>
#endif

#include <tcpdump-stdinc.h>

#include <string.h>

char *
strsep(char **stringp, const char *delim)
{
	register char *s;
	register const char *spanp;
	register int c, sc;
	char *tok;

	if ((s = *stringp) == NULL)
		return (NULL);
	for (tok = s;;) {
		c = *s++;
		spanp = delim;
		do {
			if ((sc = *spanp++) == c) {
				if (c == 0)
					s = NULL;
				else
					s[-1] = 0;
				*stringp = s;
				return (tok);
			}
		} while (sc != 0);
	}
	/* NOTREACHED */
}
