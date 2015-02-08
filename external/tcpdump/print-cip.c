

#ifndef lint
static const char rcsid[] _U_ =
    "@(#) $Header: /tcpdump/master/tcpdump/print-cip.c,v 1.25.2.1 2005/07/07 01:24:34 guy Exp $ (LBL)";
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include <tcpdump-stdinc.h>

#include <stdio.h>
#include <pcap.h>

#include "interface.h"
#include "addrtoname.h"
#include "ethertype.h"
#include "ether.h"

#define RFC1483LLC_LEN	8

static unsigned char rfcllc[] = {
	0xaa,	/* DSAP: non-ISO */
	0xaa,	/* SSAP: non-ISO */
	0x03,	/* Ctrl: Unnumbered Information Command PDU */
	0x00,	/* OUI: EtherType */
	0x00,
	0x00 };

static inline void
cip_print(int length)
{
	/*
	 * There is no MAC-layer header, so just print the length.
	 */
	printf("%d: ", length);
}

u_int
cip_if_print(const struct pcap_pkthdr *h, const u_char *p)
{
	u_int caplen = h->caplen;
	u_int length = h->len;
	u_short extracted_ethertype;

	if (memcmp(rfcllc, p, sizeof(rfcllc))==0 && caplen < RFC1483LLC_LEN) {
		printf("[|cip]");
		return (0);
	}

	if (eflag)
		cip_print(length);

	if (memcmp(rfcllc, p, sizeof(rfcllc)) == 0) {
		/*
		 * LLC header is present.  Try to print it & higher layers.
		 */
		if (llc_print(p, length, caplen, NULL, NULL,
		    &extracted_ethertype) == 0) {
			/* ether_type not known, print raw packet */
			if (!eflag)
				cip_print(length);
			if (extracted_ethertype) {
				printf("(LLC %s) ",
			       etherproto_string(htons(extracted_ethertype)));
			}
			if (!suppress_default_print)
				default_print(p, caplen);
		}
	} else {
		/*
		 * LLC header is absent; treat it as just IP.
		 */
		ip_print(gndo, p, length);
	}

	return (0);
}


