

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/pcap_dump_ftell.c,v 1.1.2.1 2005/06/03 22:10:17 guy Exp $ (LBL)";
#endif

#include <stdio.h>
#include <pcap.h>

#include "pcap-missing.h"

long
pcap_dump_ftell(pcap_dumper_t *p)
{
	return (ftell((FILE *)p));
}
