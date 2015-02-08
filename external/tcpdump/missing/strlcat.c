
/*	$NetBSD: strlcat.c,v 1.5 1999/09/20 04:39:47 lukem Exp $	*/
/*	from OpenBSD: strlcat.c,v 1.2 1999/06/17 16:28:58 millert Exp 	*/


#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/missing/strlcat.c,v 1.5 2003/11/16 09:36:51 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H 
#include <config.h>
#endif

#include <tcpdump-stdinc.h>

#include <string.h>

size_t
strlcat(char *dst, const char *src, size_t siz)
{
	register char *d = dst;
	register const char *s = src;
	register size_t n = siz;
	size_t dlen;

	/* Find the end of dst and adjust bytes left but don't go past end */
	while (*d != '\0' && n-- != 0)
		d++;
	dlen = d - dst;
	n = siz - dlen;

	if (n == 0)
		return(dlen + strlen(s));
	while (*s != '\0') {
		if (n != 1) {
			*d++ = *s;
			n--;
		}
		s++;
	}
	*d = '\0';

	return(dlen + (s - src));	/* count does not include NUL */
}
