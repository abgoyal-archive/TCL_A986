

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/setsignal.c,v 1.11 2003/11/16 09:36:42 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <signal.h>
#ifdef HAVE_SIGACTION
#include <string.h>
#endif

#ifdef HAVE_OS_PROTO_H
#include "os-proto.h"
#endif

#include "setsignal.h"

RETSIGTYPE
(*setsignal (int sig, RETSIGTYPE (*func)(int)))(int)
{
#ifdef HAVE_SIGACTION
	struct sigaction old, new;

	memset(&new, 0, sizeof(new));
	new.sa_handler = func;
	if (sigaction(sig, &new, &old) < 0)
		return (SIG_ERR);
	return (old.sa_handler);

#else
#ifdef HAVE_SIGSET
	return (sigset(sig, func));
#else
	return (signal(sig, func));
#endif
#endif
}

