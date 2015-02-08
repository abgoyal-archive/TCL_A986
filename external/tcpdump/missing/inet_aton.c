

/* $Id: inet_aton.c,v 1.6 2003/11/16 09:36:49 guy Exp $ */

#ifndef lint
static const char rcsid[] _U_ =
     "@(#) $Header: /tcpdump/master/tcpdump/missing/inet_aton.c,v 1.6 2003/11/16 09:36:49 guy Exp $";
#endif

#include <tcpdump-stdinc.h>


#ifndef INADDR_NONE
#define INADDR_NONE 0xffffffff
#endif

int
inet_aton(const char *cp, struct in_addr *addr)
{
  addr->s_addr = inet_addr(cp);
  return (addr->s_addr == INADDR_NONE) ? 0 : 1;
}
