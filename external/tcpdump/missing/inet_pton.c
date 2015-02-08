

/* $Id: inet_pton.c,v 1.6 2003/11/16 09:36:51 guy Exp $ */

#ifndef lint
static const char rcsid[] _U_ =
     "@(#) $Header: /tcpdump/master/tcpdump/missing/inet_pton.c,v 1.6 2003/11/16 09:36:51 guy Exp $";
#endif

#include <tcpdump-stdinc.h>

#include <errno.h>

int
inet_pton(int af, const char *src, void *dst)
{
    if (af != AF_INET) {
	errno = EAFNOSUPPORT;
	return -1;
    }
    return inet_aton (src, dst);
}
