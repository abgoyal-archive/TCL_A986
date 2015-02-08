

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-raw.c,v 1.41 2003/11/16 09:36:34 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <tcpdump-stdinc.h>

#include <pcap.h>
#include <stdio.h>
#include <string.h>

#include "addrtoname.h"
#include "interface.h"


u_int
raw_if_print(const struct pcap_pkthdr *h, const u_char *p)
{
	if (eflag)
		printf("ip: ");

	ipN_print(p, h->len);

	return (0);
}
