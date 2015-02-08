
#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/libpcap/pcap-null.c,v 1.21 2003/11/15 23:24:03 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/param.h>			/* optionally get BSD define */

#include <string.h>

#ifdef HAVE_OS_PROTO_H
#include "os-proto.h"
#endif

#include "pcap-int.h"

static char nosup[] = "live packet capture not supported on this system";

pcap_t *
pcap_open_live(const char *device, int snaplen, int promisc, int to_ms,
    char *ebuf)
{
	(void)strlcpy(ebuf, nosup, PCAP_ERRBUF_SIZE);
	return (NULL);
}

int
pcap_platform_finddevs(pcap_if_t **alldevsp, char *errbuf)
{
	return (0);
}
