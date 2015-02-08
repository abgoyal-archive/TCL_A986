

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/machdep.c,v 1.13 2003/12/15 03:53:21 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#ifndef HAVE___ATTRIBUTE__
#define __attribute__(x)
#endif /* HAVE___ATTRIBUTE__ */

#ifdef __osf__
#include <sys/sysinfo.h>
#include <sys/proc.h>

#if !defined(HAVE_SNPRINTF)
int snprintf(char *, size_t, const char *, ...)
     __attribute__((format(printf, 3, 4)));
#endif /* !defined(HAVE_SNPRINTF) */
#endif /* __osf__ */

#include "machdep.h"

int
abort_on_misalignment(char *ebuf _U_, size_t ebufsiz _U_)
{
#ifdef __osf__
	static int buf[2] = { SSIN_UACPROC, UAC_SIGBUS };

	if (setsysinfo(SSI_NVPAIRS, (caddr_t)buf, 1, 0, 0) < 0) {
		(void)snprintf(ebuf, ebufsiz, "setsysinfo: errno %d", errno);
		return (-1);
	}
#endif
	return (0);
}
