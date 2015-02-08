
/*	$NetBSD: strlcpy.c,v 1.5 1999/09/20 04:39:47 lukem Exp $	*/
/*	from OpenBSD: strlcpy.c,v 1.4 1999/05/01 18:56:41 millert Exp 	*/


#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/missing/strlcpy.c,v 1.5 2003/11/16 09:36:52 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H 
#include <config.h>
#endif

#include <tcpdump-stdinc.h>

#include <string.h>

size_t
strlcpy(char *dst, const char *src, size_t siz)
{
	register char *d = dst;
	register const char *s = src;
	register size_t n = siz;

	/* Copy as many bytes as will fit */
	if (n != 0 && --n != 0) {
		do {
			if ((*d++ = *s++) == 0)
				break;
		} while (--n != 0);
	}

	/* Not enough room in dst, add NUL and traverse rest of src */
	if (n == 0) {
		if (siz != 0)
			*d = '\0';		/* NUL-terminate dst */
		while (*s++)
			;
	}

	return(s - src - 1);	/* count does not include NUL */
}
